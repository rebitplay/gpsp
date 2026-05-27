/* gameplaySP
 *
 * Copyright (C) 2025 David Guillen Fandos <david@davidgf.net>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

// Here we emulate certain serial protocols to allow for netplay.
// Since real link cable emulation is very hard, here we partially emulate
// the other devices (GBAs) and using some fake data and real data
// recreate some serial protocols.

#include <assert.h>
#include "common.h"

// Debug print logic:
//#define SERIALPROTO_DEBUG 1
#ifdef SERIALPROTO_DEBUG
  #define SRPT_DEBUG_LOG(...) printf(__VA_ARGS__)
#else
  #define SRPT_DEBUG_LOG(...)
#endif

/* Always-on lightweight state-transition trace for AW protocol debugging.
   Uses printf which maps to console.log in Emscripten. */
#define AW_TRACE(...) printf(__VA_ARGS__)

#define MAX_QPACK             128    // 1 packet per frame (~2 second buffer, maybe too big)
#define MAX_FPACK             512    // ~2 words per frame (can accumulate up to 256 words per frame)


void netpacket_send(uint16_t client_id, const void *buf, size_t len);
void netpacket_poll_receive();

static void pack16(u32 *buf, const u16 *data, size_t wcnt) {
  u32 i;
  for (i = 0; i < (wcnt+1)/2; i++)
    buf[i] = netorder32((data[i*2] << 16) | (data[i*2+1]));
}
static void unpack16(u16 *buf, const u32 *data, size_t wcnt) {
  u32 i;
  for (i = 0; i < wcnt; i++)
    buf[i] = (netorder32(data[i/2])) >> ((i & 1) ? 0 : 16);
}

int ismemzero(const void *ptr, size_t bytes) {
  const u8 *p = (u8*)ptr;
  while (bytes--) {
    if (*p++)
      return 0;
  }
  return 1;
}

static union {
  struct {
    struct {
      u16 data[MAX_QPACK][8];
      u16 state;
      u8 count;                  // Number of queued packets
      u8 recvd;                  // Whether we are sending any data
      u8 timeout;                // Number of frames since last heard from.
    } peer[4];                   // One of them not used really

    u16 checksum;
    u8 offset;                   // Current frame offset
    u8 hscnt;                    // Number of handshake tokens sent
    unsigned frcnt;              // Frame counter for events.
  } poke;

  struct {
    struct {
      u8 state;                  // Device status (packet vs sync)
      u8 pstate;                 // Parsing state (packet parsing FSM)
      u16 data[MAX_FPACK];
      u16 count;                 // Number of queued bytes (full packets
      u8 recvd;                  // Whether we are sending any data
      u8 timeout;                // Number of frames since last heard from.
      u16 heartbeat_cmd;         // Latest heartbeat command from peer (consumed after one read)
    } peer[4];                   // One of them not used really

    u16 lastcmd;
    unsigned frcnt;              // Frame counter for events.
    u32 send_count;              // Total packets sent (diagnostic)
    u32 recv_count;              // Total packets received (diagnostic)
    u32 update_count;            // Total serialaw_update calls (diagnostic)
  } aw;
} serstate;

// Serial-Poke: emulates (via fakes) the pokemon serial protocol.
//
// The protocol has two phases: handshake and data exchange.
// During handshake clients are discovered using magic constants.
// Data exchange uses some frame format (1+8 half-words) to exchange data
// with some relaxed rules on latency. Frames must start with a checksum
// word followed by 8 data words.
// For masters we simulate some empty frames when no data is available,
// for slaves we simulate a fake master at a rate of 9 irqs / frame.

#define NET_SERPOKE_HEADER          0x4d504b31   // MPK1 (Multiplayer Pokemon)

// Internal states
#define STATE_PREINIT        0
#define STATE_HANDSHAKE      1
#define STATE_CONNECTED      2

#define SEQ_HANDSHAKE_TOKENS    18
#define SLAVE_IRQ_CYCLES_C   28672   // Aproximately every 1.7ms, ~9.5 per frame.
#define SLAVE_IRQ_CYCLES_H  280064   // Aproximately every frame (16.66ms).
#define MAX_FRAME_TIMEOUT      240   // After four seconds consider the peer gone.

// Pokemon protocol constants
#define MASTER_HANDSHAKE    0x8FFF
#define SLAVE_HANDSHAKE     0xB9A0   // Other games use 0xA6C0 or similar.


void serialproto_reset(void) {
  u32 i;
  SRPT_DEBUG_LOG("Reset serial-proto state\n");
  memset(&serstate, 0, sizeof(serstate));
  /* CMD_NONE (0x7FFF) signals "no command" — must be set explicitly since
     memset zeroes everything.  Without this the slave would see a spurious
     zero command on the first fake IRQ after reset. */
  for (i = 0; i < 4; i++)
    serstate.aw.peer[i].heartbeat_cmd = 0x7FFF;
}

static void serialpoke_senddata(u16 state, const u16 *packet) {
  u32 flags = state | (packet ? 0x80000000 : 0);
  u32 pkt[6] = {
    netorder32(NET_SERPOKE_HEADER),  // Header magic
    netorder32(flags),               // Current device state
    0, 0, 0, 0
  };
  if (packet)
    pack16(&pkt[2], packet, 8);

  netpacket_send(RETRO_NETPACKET_BROADCAST, pkt, sizeof(pkt));
}

// Called when serial master schedules a serial transfer.
void serialpoke_master_send(void) {
  u32 i;
  u16 mvalue = read_ioreg(REG_SIOMLT_SEND);

  write_ioreg(REG_SIOMULTI0, mvalue);   // echo sent value

  if (serstate.poke.peer[0].state == STATE_PREINIT) {
    if (mvalue == SLAVE_HANDSHAKE) {
      // Move to state handshake. Signal that to peers.
      serstate.poke.peer[0].state = STATE_HANDSHAKE;
      for (i = 1; i <= 3; i++)
        write_ioreg(REG_SIOMULTI0 + i, 0);
      // Send new state to peers.
      serialpoke_senddata(serstate.poke.peer[0].state, NULL);
    }
  }
  else if (serstate.poke.peer[0].state == STATE_HANDSHAKE) {
    // Check if our peers are ready.
    for (i = 1; i <= 3; i++)
      write_ioreg(REG_SIOMULTI0 + i,
        serstate.poke.peer[i].state == STATE_HANDSHAKE ? SLAVE_HANDSHAKE : 0xFFFF);

    // Move to connection state.
    if (mvalue == MASTER_HANDSHAKE) {
      SRPT_DEBUG_LOG("Master handshake detected, moving to connected mode!\n");
      serstate.poke.peer[0].state = STATE_CONNECTED;
      serstate.poke.checksum = 0;
      serstate.poke.offset = 0;
      serstate.poke.hscnt = 0;
      for (i = 1; i <= 3; i++)
        serstate.poke.peer[i].count = 0;    // Drop data packets
    }

    // Keeps sending updates really.
    serialpoke_senddata(serstate.poke.peer[0].state, NULL);
  } else {
    // Detect if we are trying to re-start a handshake, a bit rough.
    if (mvalue != SLAVE_HANDSHAKE)
      serstate.poke.hscnt = 0;
    else {
      if (++serstate.poke.hscnt > SEQ_HANDSHAKE_TOKENS) {
        SRPT_DEBUG_LOG("Detected handshake, switching state!\n");
        serstate.poke.peer[0].state = STATE_HANDSHAKE;
        serstate.poke.offset = 0;
        serstate.poke.checksum = 0;
        serstate.poke.hscnt = 0;
        // Write handshake values to peer slots so game sees correct data
        for (i = 1; i <= 3; i++)
          write_ioreg(REG_SIOMULTI0 + i,
            serstate.poke.peer[i].state == STATE_HANDSHAKE ? SLAVE_HANDSHAKE : 0xFFFF);
        // Notify peer immediately about state change
        serialpoke_senddata(serstate.poke.peer[0].state, NULL);
        return;
      }
    }

    // 1 checksum word, then 8 data words.
    if (serstate.poke.offset == 0) {
      for (i = 1; i <= 3; i++) {
        write_ioreg(REG_SIOMULTI0 + i, serstate.poke.checksum);
        // Check whether there's a packet ready to be transferred.
        serstate.poke.peer[i].recvd = serstate.poke.peer[i].count > 0;
      }
      serstate.poke.offset++;
      serstate.poke.checksum = 0;
    }
    else {
      serstate.poke.peer[0].data[0][serstate.poke.offset-1] = mvalue;
      serstate.poke.checksum += mvalue;

      for (i = 1; i <= 3; i++) {
        if (serstate.poke.peer[i].recvd) {
          u16 w = serstate.poke.peer[i].data[0][serstate.poke.offset-1];
          serstate.poke.checksum += w;
          write_ioreg(REG_SIOMULTI0 + i, w);
        }
        else
          write_ioreg(REG_SIOMULTI0 + i, 0);
      }

      if (serstate.poke.offset++ >= 8) {
        // Send packet data
        if (ismemzero(serstate.poke.peer[0].data[0], sizeof(serstate.poke.peer[0].data[0]))) {
          SRPT_DEBUG_LOG("Skip empty frame, just sending status instead.\n");
          serialpoke_senddata(STATE_CONNECTED, NULL);
        } else {
          SRPT_DEBUG_LOG("Sending packet as master [%04x, %04x, ...]\n",
                         serstate.poke.peer[0].data[0][0], serstate.poke.peer[0].data[0][1]);
          serialpoke_senddata(STATE_CONNECTED, serstate.poke.peer[0].data[0]);
        }

        serstate.poke.offset = 0;
        // Consume data packet.
        for (i = 1; i <= 3; i++) {
          if (serstate.poke.peer[i].recvd) {
            if (--serstate.poke.peer[i].count) {
              memmove(serstate.poke.peer[i].data[0],
                      serstate.poke.peer[i].data[1],
                      serstate.poke.peer[i].count * 8 * sizeof(u16));
            }
          }
        }
      }
    }
  }
}

void serialpoke_frame_update(void) {
  u32 i;
  for (i = 0; i <= 3; i++) {
    if (i != netplay_client_id)
      if (serstate.poke.peer[i].timeout++ >= MAX_FRAME_TIMEOUT)
        memset(&serstate.poke.peer[i], 0, sizeof(serstate.poke.peer[i]));
  }
}

bool serialpoke_update(unsigned cycles) {
  u32 i;

  // Poll network/bridge packets on each update tick so incoming
  // state/data packets are available with minimum delay.
  netpacket_poll_receive();

  serstate.poke.frcnt += cycles;

  // During handshake we tipically receive one serial transaction per frame.
  // During connection it's ~9 per frame.
  const u32 ev_cycles = (serstate.poke.peer[0].state == STATE_CONNECTED) ?
                        SLAVE_IRQ_CYCLES_C : SLAVE_IRQ_CYCLES_H;

  // Raise an IRQ periodically to pretend there's a master.
  if (netplay_client_id && serstate.poke.frcnt > ev_cycles) {
    serstate.poke.frcnt = 0;

    // We simply copy the send register to our reception register for consistency.
    write_ioreg(REG_SIOMULTI0 + netplay_client_id, read_ioreg(REG_SIOMLT_SEND));

    // Check which state the master is in and try to act accordingly.
    if (serstate.poke.peer[0].state == STATE_PREINIT)
      return false; // Does nothing, no data will be sent.

    else if (serstate.poke.peer[0].state == STATE_HANDSHAKE) {
      // Move the client to the PREINIT state, then to HANDSHAKE if it replies correctly.
      serstate.poke.peer[netplay_client_id].state = STATE_PREINIT;
      // Check if the client wants to send a handshake.
      if (read_ioreg(REG_SIOMLT_SEND) == SLAVE_HANDSHAKE)
        // Move the HANDSHAKE state!
        serstate.poke.peer[netplay_client_id].state = STATE_HANDSHAKE;

      // Emulate other values (either handshake or zero should do it).
      for (i = 0; i <= 3; i++)
        if (i != netplay_client_id)
          write_ioreg(REG_SIOMULTI0 + i,
            serstate.poke.peer[i].state == STATE_HANDSHAKE ? SLAVE_HANDSHAKE : 0xFFFF);

      // Simply send updates periodically to inform about our state.
      serialpoke_senddata(serstate.poke.peer[netplay_client_id].state, NULL);
      return true;
    }
    else {
      // Connected mode, if we are still in handshake, simulate a MASTER sync
      if (serstate.poke.peer[netplay_client_id].state == STATE_HANDSHAKE) {
        SRPT_DEBUG_LOG("Triggering IRQ for master handshake (handshake->connected).\n");
        serstate.poke.peer[netplay_client_id].state = STATE_CONNECTED;
        serstate.poke.checksum = 0;
        serstate.poke.offset = 0;
        serstate.poke.hscnt = 0;

        write_ioreg(REG_SIOMULTI0, MASTER_HANDSHAKE);
        for (i = 1; i <= 3; i++)
          if (i != netplay_client_id)
            write_ioreg(REG_SIOMULTI0 + i,
              serstate.poke.peer[i].state == STATE_HANDSHAKE ? SLAVE_HANDSHAKE : 0xFFFF);

        serialpoke_senddata(serstate.poke.peer[netplay_client_id].state, NULL);
        return true;
      }
      else if (serstate.poke.peer[netplay_client_id].state == STATE_CONNECTED) {
        u16 nw = read_ioreg(REG_SIOMLT_SEND);

        if (nw != SLAVE_HANDSHAKE)
          serstate.poke.hscnt = 0;
        else {
          if (++serstate.poke.hscnt > SEQ_HANDSHAKE_TOKENS) {
            SRPT_DEBUG_LOG("Detected handshake, switching state!\n");
            serstate.poke.peer[netplay_client_id].state = STATE_HANDSHAKE;
            serstate.poke.offset = 0;
            serstate.poke.checksum = 0;
            serstate.poke.hscnt = 0;
            // Notify peer immediately about state change
            serialpoke_senddata(serstate.poke.peer[netplay_client_id].state, NULL);
            return true;  // Fire IRQ so game processes state change
          }
        }

        if (serstate.poke.offset == 0) {
          for (i = 0; i <= 3; i++) {
            if (i == netplay_client_id)
              serstate.poke.peer[i].count = 1;

            write_ioreg(REG_SIOMULTI0 + i, serstate.poke.checksum);
            // Check whether there's a packet ready to be transferred.
            serstate.poke.peer[i].recvd = serstate.poke.peer[i].count > 0;
            SRPT_DEBUG_LOG("Client %d has %d pending packets\n", i, serstate.poke.peer[i].count);
          }
          serstate.poke.checksum = 0;
          serstate.poke.offset++;
        }
        else {
          serstate.poke.peer[netplay_client_id].data[0][serstate.poke.offset-1] = nw;

          for (i = 0; i <= 3; i++) {
            if (serstate.poke.peer[i].recvd) {
              u16 w = serstate.poke.peer[i].data[0][serstate.poke.offset-1];
              write_ioreg(REG_SIOMULTI0 + i, w);
              serstate.poke.checksum += w;
            }
            else
              write_ioreg(REG_SIOMULTI0 + i, 0);
          }

          if (serstate.poke.offset++ >= 8) {
            // Send packet data
            if (ismemzero(serstate.poke.peer[netplay_client_id].data[0],
                          sizeof(serstate.poke.peer[netplay_client_id].data[0]))) {
              SRPT_DEBUG_LOG("Skip empty frame, just sending status instead.\n");
              serialpoke_senddata(STATE_CONNECTED, NULL);
            } else {
              SRPT_DEBUG_LOG("Sending packet as slave [%04x, %04x, ...]\n",
                             serstate.poke.peer[netplay_client_id].data[0][0],
                             serstate.poke.peer[netplay_client_id].data[0][1]);
              serialpoke_senddata(STATE_CONNECTED, serstate.poke.peer[netplay_client_id].data[0]);
            }

            serstate.poke.offset = 0;
            for (i = 0; i <= 3; i++) {
              if (serstate.poke.peer[i].recvd) {
                if (--serstate.poke.peer[i].count)
                  memmove(serstate.poke.peer[i].data[0],
                          serstate.poke.peer[i].data[1],
                          serstate.poke.peer[i].count * 8 * sizeof(u16));
              }
            }
          }
        }
        return true;
      }
    }
  }
  return false;
}

void serialpoke_net_receive(const void* buf, size_t len, uint16_t client_id) {
  // MPK1 header, sanity checking.
  const u32 *pkt = (u32*)buf;
  if (len == 24 && netorder32(pkt[0]) == NET_SERPOKE_HEADER) {
    const unsigned count = serstate.poke.peer[client_id].count;
    const u32 flags = netorder32(pkt[1]);

    serstate.poke.peer[client_id].timeout = 0;

    serstate.poke.peer[client_id].state = flags & 0xFFFF;
    SRPT_DEBUG_LOG("Received valid packet from client %d (state: %d)\n",
                   client_id, serstate.poke.peer[client_id].state);

    if ((flags & 0x80000000) && count < MAX_QPACK) {
      unpack16(serstate.poke.peer[client_id].data[count], &pkt[2], 8);
      serstate.poke.peer[client_id].count++;
    }
    else if ((flags & 0x80000000)) {
      SRPT_DEBUG_LOG("Packet dropped!\n");
    }
  }
}

// Serial-AdvWrs: emulates (via fakes) the advance-wars serial protocol.
//
// The protocol uses some commands and has some packets (variable length)
// Tricky phase is the intersync phase where real keystrokes are sent.

#define NET_SERADWR_HEADER          0x4d415731   // MAW1 (Multiplayer AdvWar)

#define STATE_PACKETXG        0        // Packet processing mode
#define STATE_SYNC            1        // Sync mode (waiting for other clients)
#define STATE_INTERSYNC       2        // Sync transmission mode ("real time" key exchange)

#define PSTATE_COMMANDS       0
#define PSTATE_PACKET_HDR     1
#define PSTATE_PACKET_BODY    2

#define SLAVE_IRQ_CYCLES_2P   139264   // Aproximately every 8.3ms, twice per frame.

#define CMD_NONE       0x7FFF
#define CMD_NOP        0x5FFF

#define CMD_SYNC       (serial_mode == SERIAL_MODE_SERIAL_AW1 ? 0x5678 : 0x9ABC)
#define PACK_TAIL_SZ   (serial_mode == SERIAL_MODE_SERIAL_AW1 ? 1 : 2)

static void serialaw_senddata(u16 cmd, u8 state, const u16 *packet, size_t wcnt) {
  u32 flags = (cmd << 16) | (state << 8) | wcnt;
  u32 pkt[2 + 128] = {
    netorder32(NET_SERADWR_HEADER),  // Header magic
    netorder32(flags),               // Current device state
  };
  pack16(&pkt[2], packet, wcnt);

  serstate.aw.send_count++;
  if (wcnt > 0) {
    static int aw_data_send_count = 0;
    if (aw_data_send_count++ < 20)
      AW_TRACE("[AW-DATA-TX] cmd=%04x st=%d cnt=%zu myid=%d\n",
               cmd, state, wcnt, netplay_client_id);
  }
  netpacket_send(RETRO_NETPACKET_BROADCAST, pkt, 8 + wcnt * 2);
}

static bool empty_awpeers() {
  u32 i;
  for (i = 0; i <= 3; i++)
    if (i != netplay_client_id)
      if (serstate.aw.peer[i].count)
        return false;
  return true;
}

static u16 process_awpeer_val(u32 i) {
  // Send pending or ongoing frames.
  if (!serstate.aw.peer[i].recvd && serstate.aw.peer[i].count)
    serstate.aw.peer[i].recvd = 1;     // Start sending next frame

  if (serstate.aw.peer[i].recvd) {
    // Buffer contains: Packet-Size + payload (cmd, internal-size, word0, word1... wordN-1, wordN)

    const u16 numw = serstate.aw.peer[i].data[0];
    if (serstate.aw.peer[i].recvd >= numw) {
      u16 ret = serstate.aw.peer[i].data[serstate.aw.peer[i].recvd];
      serstate.aw.peer[i].recvd = 0;
      serstate.aw.peer[i].count -= (numw+1);
      memmove(&serstate.aw.peer[i].data[0], &serstate.aw.peer[i].data[numw+1],
              serstate.aw.peer[i].count * sizeof(u16));
      return ret;
    }
    else
      return serstate.aw.peer[i].data[serstate.aw.peer[i].recvd++];
  }

  return 0;
}

static u16 process_awpeer(u32 i) {
  // Send pending or ongoing frames.
  if (!serstate.aw.peer[i].recvd && serstate.aw.peer[i].count)
    serstate.aw.peer[i].recvd = 1;     // Start sending next frame

  if (serstate.aw.peer[i].recvd) {
    // Buffer contains: Packet-Size + payload (cmd, internal-size, word0, word1... wordN-1, wordN)

    const u16 numw = serstate.aw.peer[i].data[0];
    if (serstate.aw.peer[i].recvd > numw) {
      serstate.aw.peer[i].recvd = 0;
      serstate.aw.peer[i].count -= (numw+1);
      memmove(&serstate.aw.peer[i].data[0], &serstate.aw.peer[i].data[numw+1],
              serstate.aw.peer[i].count * sizeof(u16));
      return CMD_NONE;
    }
    else
      return serstate.aw.peer[i].data[serstate.aw.peer[i].recvd++];
  }
else {
      /* Return the latest heartbeat command if available, consuming it so
         each value is seen exactly once (prevents stale-value corruption).
         If no heartbeat is pending, fall back to CMD_SYNC when the peer is
         in SYNC state, or CMD_NONE as the idle default. */
      u16 hb = serstate.aw.peer[i].heartbeat_cmd;
      if (hb != CMD_NONE) {
        serstate.aw.peer[i].heartbeat_cmd = CMD_NONE;  /* consume */
        return hb;
      }
      if (serstate.aw.peer[0].pstate == PSTATE_COMMANDS &&
          serstate.aw.peer[i].state == STATE_SYNC)
        return CMD_SYNC;
      return CMD_NONE;
    }
}

void serialaw_master_send(void) {
  u32 i;
  u16 mvalue = read_ioreg(REG_SIOMLT_SEND);

  write_ioreg(REG_SIOMULTI0, mvalue);   // echo sent value

  if (serstate.aw.peer[0].state == STATE_SYNC) {
    static int aw_sync_log_count = 0;
    if (aw_sync_log_count++ < 10)
      AW_TRACE("[AW-MASTER] SYNC mval=%04x peer1.st=%d\n", mvalue, serstate.aw.peer[1].state);
    if (mvalue == CMD_NONE)
      serstate.aw.peer[0].state = STATE_PACKETXG;
    else {
      // We wait for all other peers to be in SYNC state, return dummy words otherwise
      for (i = 1; i <= 3; i++)
        write_ioreg(REG_SIOMULTI0 + i, serstate.aw.peer[i].state >= STATE_SYNC ? CMD_SYNC : CMD_NONE);

      if (mvalue != CMD_SYNC && mvalue != CMD_NOP) {
        AW_TRACE("[AW-MASTER] SYNC->INTERSYNC mval=%04x\n", mvalue);
        serstate.aw.peer[0].state = STATE_INTERSYNC;
        serstate.aw.peer[0].count = 0;
        SRPT_DEBUG_LOG("Changing to INTERSYNC state\n");
      }

      serialaw_senddata(0, serstate.aw.peer[0].state, NULL, 0);
    }
  } else if (serstate.aw.peer[0].state == STATE_INTERSYNC) {
    if (mvalue >= 0x8000 && mvalue <= 0x9F00) {
      for (i = 1; i <= 3; i++)
        write_ioreg(REG_SIOMULTI0 + i, process_awpeer_val(i) ?: (mvalue & 0xFC00));
      if (mvalue & 0x3FF)
        serstate.aw.peer[0].data[serstate.aw.peer[0].count++] = mvalue;
      else {
        serialaw_senddata(serstate.aw.lastcmd, STATE_INTERSYNC, serstate.aw.peer[0].data, serstate.aw.peer[0].count);
        serstate.aw.peer[0].count = 0;
      }
      serstate.aw.lastcmd = mvalue;
    }
    else if (mvalue == CMD_NOP) {
      for (i = 1; i <= 3; i++)
        write_ioreg(REG_SIOMULTI0 + i, process_awpeer_val(i) ?: (serstate.aw.lastcmd & 0xFC00));
    }
    else if (mvalue != CMD_NOP)
      serstate.aw.peer[0].state = STATE_PACKETXG;
  } else {
    if (serstate.aw.peer[0].pstate == PSTATE_COMMANDS) {
      if ((mvalue >> 8) == 0x4f)
        serstate.aw.peer[0].pstate = PSTATE_PACKET_HDR;
      else if (mvalue == CMD_SYNC && empty_awpeers()) {
        serstate.aw.peer[0].state = STATE_SYNC;
        AW_TRACE("[AW-MASTER] -> STATE_SYNC (cmd=%04x ncl=%d)\n", mvalue, netplay_num_clients);
        SRPT_DEBUG_LOG("Moving to SYNC state\n");
        /* Immediately notify peers so the slave can see STATE_SYNC
           without waiting for the next master transfer. */
        serialaw_senddata(mvalue, STATE_SYNC, NULL, 0);
      }
      else if (mvalue == CMD_SYNC) {
        /* CMD_SYNC but peers not empty — can't transition. Log it. */
        static int sync_blocked_count = 0;
        if (sync_blocked_count++ < 10)
          AW_TRACE("[AW-MASTER] CMD_SYNC blocked: q1=%d q2=%d q3=%d\n",
                   serstate.aw.peer[1].count, serstate.aw.peer[2].count, serstate.aw.peer[3].count);
        serialaw_senddata(mvalue, STATE_PACKETXG, NULL, 0);
      }
      else {
        SRPT_DEBUG_LOG("Sending update as master %04x\n", mvalue);
        serialaw_senddata(mvalue, STATE_PACKETXG, NULL, 0);
      }
    }
    else if (serstate.aw.peer[0].pstate == PSTATE_PACKET_HDR) {
      // Receives the first word, contains the packet size.
      serstate.aw.peer[0].data[0] = mvalue & 0xFF;
      serstate.aw.peer[0].count = 1;
      serstate.aw.peer[0].pstate = PSTATE_PACKET_BODY;
    }
    else {
      // We send the N+2 words (not sure why 2 extra words are sent though).
      unsigned pktlen = serstate.aw.peer[0].data[0] + PACK_TAIL_SZ;
      serstate.aw.peer[0].data[serstate.aw.peer[0].count++] = mvalue;
      if (serstate.aw.peer[0].count == pktlen + 1) {
        // Full packet received, send it to the clients.
        serstate.aw.peer[0].pstate = PSTATE_COMMANDS;
        serialaw_senddata(0x4fff, STATE_PACKETXG, serstate.aw.peer[0].data, serstate.aw.peer[0].count);
        SRPT_DEBUG_LOG("Sending packet as master [%04x, %04x, %04x, %04x, ...] %d words\n",
                       serstate.aw.peer[0].data[0], serstate.aw.peer[0].data[1],
                       serstate.aw.peer[0].data[2], serstate.aw.peer[0].data[3], serstate.aw.peer[0].count);
      }
    }

    for (i = 1; i <= 3; i++)
      write_ioreg(REG_SIOMULTI0 + i, process_awpeer(i));
  }

  serstate.aw.lastcmd = mvalue;

  SRPT_DEBUG_LOG("Return words %04x %04x %04x %04x\n",
    read_ioreg(REG_SIOMULTI0), read_ioreg(REG_SIOMULTI1),
    read_ioreg(REG_SIOMULTI2), read_ioreg(REG_SIOMULTI3));
}

bool serialaw_update(unsigned cycles) {
  u32 i;

  static int aw_update_first = 1;
  if (aw_update_first) {
    aw_update_first = 0;
    AW_TRACE("[AW-INIT] serialaw_update first call, client_id=%d num_clients=%d serial_mode=%d\n",
             netplay_client_id, netplay_num_clients, serial_mode);
  }

  serstate.aw.update_count++;
  /* Periodic summary every ~2 seconds (assuming ~120 calls/s for slave, fewer for master) */
  if ((serstate.aw.update_count & 0xFF) == 0) {
    AW_TRACE("[AW-STAT] upd=%u tx=%u rx=%u myid=%d st=%d pst=%d "
             "peer0.st=%d peer1.st=%d q0=%d q1=%d "
             "SIO=%04x,%04x,%04x,%04x CNT=%04x\n",
             serstate.aw.update_count, serstate.aw.send_count, serstate.aw.recv_count,
             netplay_client_id,
             serstate.aw.peer[netplay_client_id].state,
             serstate.aw.peer[netplay_client_id].pstate,
             serstate.aw.peer[0].state, serstate.aw.peer[1].state,
             serstate.aw.peer[0].count, serstate.aw.peer[1].count,
             read_ioreg(REG_SIOMULTI0), read_ioreg(REG_SIOMULTI1),
             read_ioreg(REG_SIOMULTI2), read_ioreg(REG_SIOMULTI3),
             read_ioreg(REG_SIOCNT));
  }

  // AW link mode is very timing sensitive. Poll network/bridge packets
  // on each update tick so incoming words are available with minimum delay.
  netpacket_poll_receive();

  serstate.aw.frcnt += cycles;

  // During sync it's one per frame, otherwise ~2 per frame.
  const u32 ev_cycles = (serstate.aw.peer[netplay_client_id].state >= STATE_SYNC) ?
                        SLAVE_IRQ_CYCLES_H : SLAVE_IRQ_CYCLES_2P;

  // Raise an IRQ periodically to pretend there's a master.
  if (netplay_client_id && serstate.aw.frcnt > ev_cycles) {
    u16 mdata = read_ioreg(REG_SIOMLT_SEND);
    serstate.aw.frcnt = 0;

    // We simply copy the send register to our reception register for consistency.
    write_ioreg(REG_SIOMULTI0 + netplay_client_id, mdata);

    if (serstate.aw.peer[netplay_client_id].state == STATE_SYNC) {
      if (mdata == CMD_NONE)
        serstate.aw.peer[netplay_client_id].state = STATE_PACKETXG;
      else {
        // We wait for all other peers to be in SYNC state, return dummy words otherwise
        if (mdata == CMD_SYNC || mdata == CMD_NOP) {
          for (i = 0; i <= 3; i++)
            if (i != netplay_client_id)
              write_ioreg(REG_SIOMULTI0 + i, serstate.aw.peer[i].state >= STATE_SYNC ? CMD_SYNC : CMD_NONE);
        }
        else {
          SRPT_DEBUG_LOG("Changing to INTERSYNC state\n");
          serstate.aw.peer[netplay_client_id].state = STATE_INTERSYNC;
          serstate.aw.peer[netplay_client_id].count = 0;
        }
        serialaw_senddata(0, serstate.aw.peer[netplay_client_id].state, NULL, 0);
      }
    }

    // Re-evaluate state again (in case we moved to another state.
    if (serstate.aw.peer[netplay_client_id].state == STATE_INTERSYNC) {
      if (mdata >= 0x8000 && mdata <= 0x9F00) {
        for (i = 0; i <= 3; i++)
          if (i != netplay_client_id)
            write_ioreg(REG_SIOMULTI0 + i, process_awpeer_val(i) ?: (mdata & 0xFC00));
        if (mdata & 0x3FF)
          serstate.aw.peer[netplay_client_id].data[serstate.aw.peer[netplay_client_id].count++] = mdata;
        else {
          serialaw_senddata(serstate.aw.lastcmd, STATE_INTERSYNC,
                            serstate.aw.peer[netplay_client_id].data, serstate.aw.peer[netplay_client_id].count);
          serstate.aw.peer[netplay_client_id].count = 0;
        }
        serstate.aw.lastcmd = mdata;
      }
      else if (mdata == CMD_NOP) {
        for (i = 0; i <= 3; i++)
          if (i != netplay_client_id)
            write_ioreg(REG_SIOMULTI0 + i, process_awpeer_val(i) ?: (serstate.aw.lastcmd & 0xFC00));
      }
      else
        serstate.aw.peer[netplay_client_id].state = STATE_PACKETXG;

    }
    else if (serstate.aw.peer[netplay_client_id].state == STATE_PACKETXG) {
      if (serstate.aw.peer[netplay_client_id].pstate == PSTATE_COMMANDS) {
        if ((mdata >> 8) == 0x4f)
          serstate.aw.peer[netplay_client_id].pstate = PSTATE_PACKET_HDR;
        else if (mdata == CMD_SYNC && empty_awpeers()) {
          serstate.aw.peer[netplay_client_id].state = STATE_SYNC;
          AW_TRACE("[AW-SLAVE] -> STATE_SYNC (cmd=%04x ncl=%d)\n", mdata, netplay_num_clients);
          SRPT_DEBUG_LOG("Moving to SYNC state\n");
          /* Immediately notify peers about the SYNC transition. */
          serialaw_senddata(mdata, STATE_SYNC, NULL, 0);
        }
        else {
          SRPT_DEBUG_LOG("Sending update as client %04x\n", mdata);
          serialaw_senddata(mdata, STATE_PACKETXG, NULL, 0);
        }
      }
      else if (serstate.aw.peer[netplay_client_id].pstate == PSTATE_PACKET_HDR) {
        // Receives the first word, contains the packet size.
        serstate.aw.peer[netplay_client_id].data[0] = mdata & 0xFF;
        serstate.aw.peer[netplay_client_id].count = 1;
        serstate.aw.peer[netplay_client_id].pstate = PSTATE_PACKET_BODY;
      }
      else {
        // We send the N+2 words (not sure why 2 extra words are sent though).
        unsigned pktlen = serstate.aw.peer[netplay_client_id].data[0] + PACK_TAIL_SZ;
        serstate.aw.peer[netplay_client_id].data[serstate.aw.peer[netplay_client_id].count++] = mdata;
        if (serstate.aw.peer[netplay_client_id].count == pktlen + 1) {
          // Full packet received, send it to the clients.
          serstate.aw.peer[netplay_client_id].pstate = PSTATE_COMMANDS;
          serialaw_senddata(0x4fff, STATE_PACKETXG, serstate.aw.peer[netplay_client_id].data, serstate.aw.peer[netplay_client_id].count);
          SRPT_DEBUG_LOG("Sending packet as client [%04x, %04x, %04x, %04x, ...] %d words\n",
                         serstate.aw.peer[netplay_client_id].data[0], serstate.aw.peer[netplay_client_id].data[1],
                         serstate.aw.peer[netplay_client_id].data[2], serstate.aw.peer[netplay_client_id].data[3],
                         serstate.aw.peer[netplay_client_id].count);

        }
      }

      for (i = 0; i <= 3; i++)
        if (i != netplay_client_id)
          write_ioreg(REG_SIOMULTI0 + i, process_awpeer(i));
    }

    SRPT_DEBUG_LOG("Fake receive serial %04x %04x %04x %04x\n",
      read_ioreg(REG_SIOMULTI0), read_ioreg(REG_SIOMULTI1),
      read_ioreg(REG_SIOMULTI2), read_ioreg(REG_SIOMULTI3));

    /* Clear the start/busy bit in SIOCNT, just like real GBA hardware does
     * when a multiplayer transfer completes.  Without this the game may
     * think the transfer is still in progress and refuse to process the
     * received SIOMULTI data. */
    write_ioreg(REG_SIOCNT, (read_ioreg(REG_SIOCNT) & ~0x80));

    return true;
  }

  return false;
}

void serialaw_net_receive(const void* buf, size_t len, uint16_t client_id) {
  // MAW1 header, sanity checking.
  const u32 *pkt = (u32*)buf;
  if (len >= 8 && netorder32(pkt[0]) == NET_SERADWR_HEADER) {
    const u32 flags = netorder32(pkt[1]);
    const u16 cmd = flags >> 16;
    const u16 ste = (flags >> 8) & 0xff;    // Peer state.
    const u16 cnt = flags & 0x00ff;         // Number of words to follow.

    static int aw_recv_log_count = 0;
    if (aw_recv_log_count++ < 30)
      AW_TRACE("[AW-RECV] from=%d st=%d cmd=%04x cnt=%d myid=%d\n",
               client_id, ste, cmd, cnt, netplay_client_id);

    serstate.aw.peer[client_id].timeout = 0;
    serstate.aw.recv_count++;

    /* When the peer changes state, discard any leftover heartbeat from
       the previous state — it would confuse the local FSM.  For example
       a game command cached while in PACKETXG must not leak through
       after the peer moves to SYNC. */
    if (ste != serstate.aw.peer[client_id].state)
      serstate.aw.peer[client_id].heartbeat_cmd = CMD_NONE;

    serstate.aw.peer[client_id].state = ste;

    /* For heartbeat packets (cnt=0) store the command so the other side
       can see it in SIOMULTI on the next transfer / fake-IRQ.  Each value
       is consumed once by process_awpeer().
       cmd==0 is a dummy placeholder used in state-notification sends
       (e.g. serialaw_senddata(0, STATE_SYNC, NULL, 0)) — it must NOT
       be forwarded or the game would see 0x0000 in SIOMULTI, which it
       interprets as corrupt data → linking error. */
    if (cnt == 0 && cmd != 0) {
      serstate.aw.peer[client_id].heartbeat_cmd = cmd;
    }

    SRPT_DEBUG_LOG("Got packet with state %d cmd %04x and size %d.\n", ste, cmd, cnt);

    if (ste == STATE_INTERSYNC) {
      if (cnt >= 2 && len == cnt * 2 + 8) {
        if (serstate.aw.peer[client_id].count + cnt + 1 < MAX_FPACK) {
          serstate.aw.peer[client_id].data[serstate.aw.peer[client_id].count++] = cnt;
          unpack16(&serstate.aw.peer[client_id].data[serstate.aw.peer[client_id].count], &pkt[2], cnt);
          serstate.aw.peer[client_id].count += cnt;

          SRPT_DEBUG_LOG("Received valid sync command client %d: %04x\n", client_id, cmd);
        }
      }
    }
    else if (ste == STATE_PACKETXG) {
      if (cnt >= 2 && len == cnt * 2 + 8) {
        if (serstate.aw.peer[client_id].count + cnt + 2 < MAX_FPACK) {
          // We insert this in the queue, adding a header for length.
          // Also the command value (should be 0x4FFF) is inserted too.
          serstate.aw.peer[client_id].data[serstate.aw.peer[client_id].count++] = cnt + 1;  // Packet + command
          serstate.aw.peer[client_id].data[serstate.aw.peer[client_id].count++] = cmd;
          unpack16(&serstate.aw.peer[client_id].data[serstate.aw.peer[client_id].count], &pkt[2], cnt);
          serstate.aw.peer[client_id].count += cnt;

          static int aw_data_recv_count = 0;
          if (aw_data_recv_count++ < 20)
            AW_TRACE("[AW-DATA-RX] from=%d cmd=%04x cnt=%d qlen=%d myid=%d\n",
                     client_id, cmd, cnt, serstate.aw.peer[client_id].count, netplay_client_id);
        }
        else {
          AW_TRACE("[AW-DROP] from=%d cnt=%d qlen=%d MAX=%d\n",
                   client_id, cnt, serstate.aw.peer[client_id].count, MAX_FPACK);
        }
      }
    }
  }
}


