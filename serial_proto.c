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

#define MAX_QPACK             128    // 1 packet per frame (~2 second buffer, maybe too big)
#define MAX_FPACK            2048    // AW link packets can burst heavily before sync settles.
#define RAW_TRACE_LEN          64


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
    } peer[4];                   // One of them not used really

    u16 lastcmd;
    unsigned frcnt;              // Frame counter for events.
    struct {
      u8 enabled;
      u8 phase;
      u8 reply_mask;
      u8 ack_mask;
      u8 pending_start;
      u8 irq_pending;
      u8 request_pending;
      u8 wait_send_write;
      u8 ack_pending;
      u8 transfer_pending;
      u8 ack_ready;
      u8 fast_ack;
      u8 ack_write_count;
      u32 seq;
      u32 pending_seq;
      u32 request_seq;
      u32 wait_cycles;
      u32 request_wait_cycles;
      u32 ack_wait_cycles;
      u32 timeout_count;
      u32 send_write_count;
      u32 fresh_send_count;
      u32 stale_reply_count;
      u32 ack_send_count;
      u32 ack_recv_count;
      u32 reply_send_count;
      u32 reply_recv_count;
      u32 bus_recv_count;
      u32 last_ack_send_seq;
      u32 last_drop_seq;
      u32 trace_count;
      u8 trace_pos;
      u16 pending_host_send;
      u16 reply[4];
      u16 last_send[4];
      u16 bus[4];
      u16 request_words[4];
      struct {
        u32 seq;
        u16 event;
        u16 extra;
        u16 words[4];
        u16 siocnt;
        u16 rcnt;
        u16 flags;
      } trace[RAW_TRACE_LEN];
    } raw;
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
  const u8 raw_enabled = serstate.aw.raw.enabled;
  u8 raw_state[sizeof(serstate.aw.raw)];
  if (raw_enabled)
    memcpy(raw_state, &serstate.aw.raw, sizeof(raw_state));

  SRPT_DEBUG_LOG("Reset serial-proto state\n");
  memset(&serstate, 0, sizeof(serstate));
  if (raw_enabled)
    memcpy(&serstate.aw.raw, raw_state, sizeof(raw_state));
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
            return false;
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

#define SLAVE_IRQ_CYCLES_PACKET 10484  // 115200bps multi-player transfer timing for four players.

#define CMD_NONE       0x7FFF
#define CMD_NOP        0x5FFF

#define CMD_SYNC       (serial_mode == SERIAL_MODE_SERIAL_AW1 ? 0x5678 : 0x9ABC)
#define PACK_TAIL_SZ   (serial_mode == SERIAL_MODE_SERIAL_AW1 ? 1 : 2)

#define NET_SERAWRW_HEADER          0x52415732   // RAW2
#define RAW_KIND_REQ                1
#define RAW_KIND_REPLY              2
#define RAW_KIND_BUS                3
#define RAW_KIND_ACK                4
#define RAW_TRACE_REQ_TX            1
#define RAW_TRACE_REQ_RX            2
#define RAW_TRACE_REPLY_TX          3
#define RAW_TRACE_REPLY_RX          4
#define RAW_TRACE_BUS_TX            5
#define RAW_TRACE_BUS_RX            6
#define RAW_TRACE_IRQ               7
#define RAW_TRACE_ACK_TX            8
#define RAW_TRACE_ACK_RX            9
#define RAW_TRACE_STALE            10
#define RAW_TRACE_SEND_WRITE       11
#define RAW_PHASE_IDLE              0
#define RAW_PHASE_WAIT_REPLIES      1
#define RAW_PHASE_WAIT_ACKS         2
#define RAW_TIMEOUT_CYCLES          33707520     // About two seconds; avoid fabricating partial link transfers.
#define RAW_READY_TIMEOUT_CYCLES    (280896 * 12) // Give guests time to publish post-IRQ send words before falling back to stale data.
#define RAW_ACK_QUIET_CYCLES        1 // Let same-burst post-IRQ send-word writes settle without a frame-scale delay.
#define RAW_POLL_CYCLES             4096

static u32 raw_transfer_cycles(void) {
  static const u32 per_word_cycles[4] = { 31457, 7864, 5242, 2621 };
  const u32 clients = netplay_num_clients ? netplay_num_clients : 3;
  const u32 slaves = clients < 1 ? 1 : (clients > 3 ? 3 : clients);
  return per_word_cycles[read_ioreg(REG_SIOCNT) & 0x3] * (slaves + 1);
}

static u8 raw_expected_mask(void) {
  u8 mask = 1;
  u32 i;
  const u32 clients = netplay_num_clients ? netplay_num_clients : 3;
  for (i = 1; i <= clients && i <= 3; i++)
    mask |= (u8)(1 << i);
  return mask;
}

static u16 raw_mul_siocnt_status(void) {
  return (u16)(0x08 | ((netplay_client_id & 3) << 4) |
               (netplay_client_id ? 0x04 : 0));
}

static void raw_write_transfer_lines(u16 value) {
  value &= 0xff4b;
  value |= 0x80;
  value |= raw_mul_siocnt_status();
  write_ioreg(REG_SIOCNT, value);
  write_ioreg(REG_RCNT, netplay_client_id ? 6 : 2);
}

static void raw_write_complete_lines(void) {
  u16 value = read_ioreg(REG_SIOCNT);
  value = (value & 0xff03) | raw_mul_siocnt_status();
  write_ioreg(REG_SIOCNT, value);
  write_ioreg(REG_RCNT, netplay_client_id ? 0x0f : 0x0b);
}

static void raw_pack_words(u32 *pkt, const u16 *words) {
  pkt[3] = netorder32(((u32)words[0] << 16) | words[1]);
  pkt[4] = netorder32(((u32)words[2] << 16) | words[3]);
}

static void raw_unpack_words(u16 *words, const u32 *pkt) {
  const u32 a = netorder32(pkt[3]);
  const u32 b = netorder32(pkt[4]);
  words[0] = a >> 16;
  words[1] = a & 0xffff;
  words[2] = b >> 16;
  words[3] = b & 0xffff;
}

static void raw_trace(u16 event, u32 seq, const u16 *words, u16 extra) {
  u32 i;
  const u32 pos = serstate.aw.raw.trace_pos % RAW_TRACE_LEN;
  const u16 flags = (u16)((serstate.aw.raw.phase << 12) |
                          (serstate.aw.raw.reply_mask << 8) |
                          (serstate.aw.raw.ack_mask << 4) |
                          (serstate.aw.raw.irq_pending ? 0x0001 : 0) |
                          (serstate.aw.raw.request_pending ? 0x0002 : 0) |
                          (serstate.aw.raw.wait_send_write ? 0x0004 : 0) |
                          (serstate.aw.raw.ack_pending ? 0x0008 : 0));

  serstate.aw.raw.trace[pos].seq = seq;
  serstate.aw.raw.trace[pos].event = event;
  serstate.aw.raw.trace[pos].extra = extra;
  for (i = 0; i <= 3; i++)
    serstate.aw.raw.trace[pos].words[i] = words ? words[i] : 0;
  serstate.aw.raw.trace[pos].siocnt = read_ioreg(REG_SIOCNT);
  serstate.aw.raw.trace[pos].rcnt = read_ioreg(REG_RCNT);
  serstate.aw.raw.trace[pos].flags = flags;

  serstate.aw.raw.trace_pos = (pos + 1) % RAW_TRACE_LEN;
  serstate.aw.raw.trace_count++;
}

static void raw_send(u8 kind, u32 seq, const u16 *words, uint16_t target) {
  u32 pkt[5] = {
    netorder32(NET_SERAWRW_HEADER),
    netorder32(kind),
    netorder32(seq),
    0,
    0,
  };
  raw_pack_words(pkt, words);
  netpacket_send(target, pkt, sizeof(pkt));
}

static void raw_send_ack(u32 seq) {
  const u16 words[4] = { 0, 0, 0, 0 };
  serstate.aw.raw.ack_send_count++;
  serstate.aw.raw.last_ack_send_seq = seq;
  raw_trace(RAW_TRACE_ACK_TX, seq, words, 0);
  raw_send(RAW_KIND_ACK, seq, words, 0);
}

static void raw_reply_to_pending_request(void);

static void raw_finish_deferred_ack(void) {
  raw_send_ack(serstate.aw.raw.pending_seq);
  serstate.aw.raw.ack_pending = 0;
  serstate.aw.raw.ack_ready = 0;
  serstate.aw.raw.ack_write_count = 0;
  serstate.aw.raw.ack_wait_cycles = 0;
  serstate.aw.raw.wait_send_write = 0;
  serstate.aw.raw.request_wait_cycles = 0;

  if (serstate.aw.raw.request_pending)
    raw_reply_to_pending_request();
}

static bool raw_client_prepare_seq(u32 seq) {
  if (serstate.aw.raw.pending_seq && seq < serstate.aw.raw.pending_seq)
    return false;

  if (seq != serstate.aw.raw.pending_seq) {
    serstate.aw.raw.pending_seq = seq;
    serstate.aw.raw.reply_mask = 0;
    memset(serstate.aw.raw.reply, 0, sizeof(serstate.aw.raw.reply));
  }

  return true;
}

static void raw_reply_to_pending_request(void) {
  u16 words[4];
  const u8 local_mask = (u8)(1 << (netplay_client_id & 3));

  serstate.aw.raw.request_words[netplay_client_id] = read_ioreg(REG_SIOMLT_SEND);
  memcpy(words, serstate.aw.raw.request_words, sizeof(words));
  serstate.aw.raw.reply[0] = words[0];
  serstate.aw.raw.reply[netplay_client_id] = words[netplay_client_id];
  serstate.aw.raw.reply_mask |= (u8)(1 | local_mask);
  serstate.aw.raw.last_send[0] = words[0];
  serstate.aw.raw.last_send[netplay_client_id] = words[netplay_client_id];
  serstate.aw.raw.reply_send_count++;
  raw_trace(RAW_TRACE_REPLY_TX, serstate.aw.raw.request_seq, words, words[netplay_client_id]);
  raw_send(RAW_KIND_REPLY, serstate.aw.raw.request_seq, words, 0);
  serstate.aw.raw.request_pending = 0;
  serstate.aw.raw.request_wait_cycles = 0;
}

static void raw_receive_request(u32 seq, const u16 *words) {
  u32 i;

  if (!raw_client_prepare_seq(seq))
    return;

  raw_write_transfer_lines(read_ioreg(REG_SIOCNT));
  for (i = 0; i <= 3; i++)
    write_ioreg(REG_SIOMULTI0 + i, 0xffff);
  raw_trace(RAW_TRACE_REQ_RX, seq, words, read_ioreg(REG_SIOMLT_SEND));
  memcpy(serstate.aw.raw.request_words, words, sizeof(serstate.aw.raw.request_words));
  serstate.aw.raw.request_words[netplay_client_id] = read_ioreg(REG_SIOMLT_SEND);
  serstate.aw.raw.request_seq = seq;
  serstate.aw.raw.reply[0] = words[0];
  serstate.aw.raw.reply_mask |= 1;
  serstate.aw.raw.transfer_pending = 1;
  serstate.aw.raw.wait_cycles = 0;
  serstate.aw.raw.request_wait_cycles = 0;

  serstate.aw.raw.request_pending = 1;
  raw_reply_to_pending_request();
}

static void raw_begin_request(u16 host_send) {
  u16 words[4] = { host_send, 0, 0, 0 };
  u32 i;

  raw_write_transfer_lines(read_ioreg(REG_SIOCNT));
  for (i = 0; i <= 3; i++)
    write_ioreg(REG_SIOMULTI0 + i, 0xffff);
  serstate.aw.raw.phase = RAW_PHASE_WAIT_REPLIES;
  serstate.aw.raw.pending_seq = ++serstate.aw.raw.seq;
  serstate.aw.raw.reply_mask = 1;
  serstate.aw.raw.ack_mask = 1;
  serstate.aw.raw.wait_cycles = 0;
  serstate.aw.raw.reply[0] = host_send;
  serstate.aw.raw.last_send[0] = host_send;
  raw_trace(RAW_TRACE_REQ_TX, serstate.aw.raw.pending_seq, words, host_send);
  raw_send(RAW_KIND_REQ, serstate.aw.raw.pending_seq, words, RETRO_NETPACKET_BROADCAST);
}

static bool raw_host_finish_acks_if_ready(void) {
  const u8 expected = raw_expected_mask();
  u32 i;

  if (netplay_client_id ||
      serstate.aw.raw.phase != RAW_PHASE_WAIT_ACKS ||
      (serstate.aw.raw.ack_mask & expected) != expected)
    return false;

  serstate.aw.raw.wait_cycles = 0;
  for (i = 0; i <= 3; i++)
    write_ioreg(REG_SIOMULTI0 + i, serstate.aw.raw.bus[i]);

  raw_write_complete_lines();
  raw_trace(RAW_TRACE_IRQ, serstate.aw.raw.pending_seq, serstate.aw.raw.bus,
            ((u16)serstate.aw.raw.reply_mask << 8) | serstate.aw.raw.ack_mask);

  if (serstate.aw.raw.pending_start) {
    serstate.aw.raw.pending_start = 0;
    raw_begin_request(serstate.aw.raw.pending_host_send);
  } else {
    serstate.aw.raw.phase = RAW_PHASE_IDLE;
  }
  return read_ioreg(REG_SIOCNT) & 0x4000;
}

bool serialaw_raw_master_start(void) {
  if (!serstate.aw.raw.enabled || netplay_client_id)
    return false;

  if (serstate.aw.raw.phase != RAW_PHASE_IDLE) {
    if (serstate.aw.raw.phase == RAW_PHASE_WAIT_ACKS &&
        !serstate.aw.raw.pending_start) {
      serstate.aw.raw.pending_host_send = read_ioreg(REG_SIOMLT_SEND);
      serstate.aw.raw.pending_start = 1;
    }
    return true;
  }

  serstate.aw.raw.pending_host_send = read_ioreg(REG_SIOMLT_SEND);
  serstate.aw.raw.pending_start = 0;
  raw_begin_request(serstate.aw.raw.pending_host_send);
  return true;
}

void serialaw_raw_send_write(u16 value) {
  if (!serstate.aw.raw.enabled)
    return;

  serstate.aw.raw.last_send[netplay_client_id & 3] = value;
  if (netplay_client_id && serstate.aw.raw.request_pending)
    serstate.aw.raw.request_words[netplay_client_id & 3] = value;
  serstate.aw.raw.send_write_count++;
  raw_trace(RAW_TRACE_SEND_WRITE, serstate.aw.raw.pending_seq, serstate.aw.raw.last_send, value);

  if (netplay_client_id && serstate.aw.raw.ack_pending) {
    serstate.aw.raw.ack_write_count++;
    serstate.aw.raw.ack_ready = 1;
    serstate.aw.raw.wait_send_write = 0;
    serstate.aw.raw.fresh_send_count++;
    serstate.aw.raw.ack_wait_cycles = 0;
    return;
  }

  if (netplay_client_id && serstate.aw.raw.wait_send_write) {
    serstate.aw.raw.wait_send_write = 0;
    serstate.aw.raw.fresh_send_count++;
    if (serstate.aw.raw.request_pending)
      raw_reply_to_pending_request();
    else
      serstate.aw.raw.request_wait_cycles = 0;
  }
}

static bool raw_complete_master_transfer(void) {
  u32 i;
  for (i = 0; i <= 3; i++) {
    u16 value = (serstate.aw.raw.reply_mask & (1 << i)) ?
      serstate.aw.raw.reply[i] : serstate.aw.raw.last_send[i];

    serstate.aw.raw.bus[i] = value;
    serstate.aw.raw.last_send[i] = value;
  }

  raw_trace(RAW_TRACE_BUS_TX, serstate.aw.raw.pending_seq, serstate.aw.raw.bus,
            ((u16)serstate.aw.raw.reply_mask << 8) | serstate.aw.raw.ack_mask);

  serstate.aw.raw.phase = RAW_PHASE_WAIT_ACKS;
  serstate.aw.raw.ack_mask = 1;
  serstate.aw.raw.wait_cycles = 0;
  raw_send(RAW_KIND_BUS, serstate.aw.raw.pending_seq, serstate.aw.raw.bus,
           RETRO_NETPACKET_BROADCAST);
  return false;
}

static bool raw_complete_client_transfer(void) {
  u32 i;

  for (i = 0; i <= 3; i++) {
    u16 value = (serstate.aw.raw.reply_mask & (1 << i)) ?
      serstate.aw.raw.reply[i] : serstate.aw.raw.last_send[i];

    serstate.aw.raw.bus[i] = value;
    serstate.aw.raw.last_send[i] = value;
    write_ioreg(REG_SIOMULTI0 + i, value);
  }

  serstate.aw.raw.irq_pending = 0;
  serstate.aw.raw.transfer_pending = 0;
  serstate.aw.raw.wait_cycles = 0;
  serstate.aw.raw.wait_send_write = 1;
  serstate.aw.raw.ack_pending = 1;
  serstate.aw.raw.ack_ready = 0;
  serstate.aw.raw.ack_write_count = 0;
  serstate.aw.raw.ack_wait_cycles = 0;
  serstate.aw.raw.request_wait_cycles = 0;
  raw_write_complete_lines();
  raw_trace(RAW_TRACE_IRQ, serstate.aw.raw.pending_seq, serstate.aw.raw.bus, read_ioreg(REG_SIOMLT_SEND));
  return read_ioreg(REG_SIOCNT) & 0x4000;
}

static bool raw_update(unsigned cycles) {
  netpacket_poll_receive();

  if (!netplay_client_id) {
    const u8 expected = raw_expected_mask();

    if (serstate.aw.raw.phase == RAW_PHASE_WAIT_REPLIES) {
      serstate.aw.raw.wait_cycles += cycles;
      if ((serstate.aw.raw.reply_mask & expected) == expected &&
          serstate.aw.raw.wait_cycles >= raw_transfer_cycles())
        return raw_complete_master_transfer();

      if (serstate.aw.raw.wait_cycles >= RAW_TIMEOUT_CYCLES) {
        serstate.aw.raw.timeout_count++;
        serstate.aw.raw.wait_cycles = 0;
      }
    }
    else if (serstate.aw.raw.phase == RAW_PHASE_WAIT_ACKS) {
      serstate.aw.raw.wait_cycles += cycles;
      if (raw_host_finish_acks_if_ready())
        return true;
      if (serstate.aw.raw.phase == RAW_PHASE_WAIT_ACKS &&
          serstate.aw.raw.wait_cycles >= RAW_TIMEOUT_CYCLES) {
        serstate.aw.raw.timeout_count++;
        serstate.aw.raw.wait_cycles = 0;
      }
    }

    return false;
  }

  if (serstate.aw.raw.wait_send_write && serstate.aw.raw.request_pending) {
    serstate.aw.raw.request_wait_cycles += cycles;
    if (serstate.aw.raw.request_wait_cycles >= RAW_READY_TIMEOUT_CYCLES) {
      serstate.aw.raw.timeout_count++;
      serstate.aw.raw.stale_reply_count++;
      serstate.aw.raw.wait_send_write = 0;
      raw_trace(RAW_TRACE_STALE, serstate.aw.raw.request_pending ?
        serstate.aw.raw.request_seq : serstate.aw.raw.pending_seq,
        serstate.aw.raw.request_pending ? serstate.aw.raw.request_words : serstate.aw.raw.bus,
        read_ioreg(REG_SIOMLT_SEND));
      if (serstate.aw.raw.request_pending)
        raw_reply_to_pending_request();
      else
        serstate.aw.raw.request_wait_cycles = 0;
    }
  }

  if (serstate.aw.raw.ack_pending) {
    serstate.aw.raw.ack_wait_cycles += cycles;
    if (serstate.aw.raw.ack_ready) {
      if (serstate.aw.raw.ack_wait_cycles >= RAW_ACK_QUIET_CYCLES)
        raw_finish_deferred_ack();
    }
    else if (serstate.aw.raw.ack_wait_cycles >= RAW_READY_TIMEOUT_CYCLES) {
      serstate.aw.raw.timeout_count++;
      serstate.aw.raw.stale_reply_count++;
      raw_trace(RAW_TRACE_STALE, serstate.aw.raw.pending_seq,
                serstate.aw.raw.bus, read_ioreg(REG_SIOMLT_SEND));
      raw_finish_deferred_ack();
    }
  }

  if (serstate.aw.raw.request_pending &&
      !serstate.aw.raw.wait_send_write &&
      !serstate.aw.raw.ack_pending) {
    serstate.aw.raw.request_wait_cycles += cycles;
    if (serstate.aw.raw.request_wait_cycles >= RAW_POLL_CYCLES)
      raw_reply_to_pending_request();
  }

  if (serstate.aw.raw.transfer_pending &&
      serstate.aw.raw.wait_cycles < RAW_TIMEOUT_CYCLES)
    serstate.aw.raw.wait_cycles += cycles;

  if (serstate.aw.raw.irq_pending) {
    if (serstate.aw.raw.transfer_pending &&
        serstate.aw.raw.wait_cycles < raw_transfer_cycles())
      return false;

    return raw_complete_client_transfer();
  }

  return false;
}

u32 serialaw_next_event(void) {
  if (!serstate.aw.raw.enabled)
    return ~0U;

  if (!netplay_client_id) {
    const u8 expected = raw_expected_mask();

    if (serstate.aw.raw.phase == RAW_PHASE_WAIT_REPLIES) {
      if ((serstate.aw.raw.reply_mask & expected) == expected) {
        const u32 transfer_cycles = raw_transfer_cycles();
        if (serstate.aw.raw.wait_cycles >= transfer_cycles)
          return 1;
        return transfer_cycles - serstate.aw.raw.wait_cycles;
      }
      return RAW_POLL_CYCLES;
    }

    if (serstate.aw.raw.phase == RAW_PHASE_WAIT_ACKS)
      return ((serstate.aw.raw.ack_mask & expected) == expected) ? 1 : RAW_POLL_CYCLES;

    return ~0U;
  }

  if (serstate.aw.raw.irq_pending) {
    if (serstate.aw.raw.transfer_pending) {
      const u32 transfer_cycles = raw_transfer_cycles();
      if (serstate.aw.raw.wait_cycles < transfer_cycles)
        return transfer_cycles - serstate.aw.raw.wait_cycles;
    }
    return 1;
  }

  if (serstate.aw.raw.transfer_pending) {
    return RAW_POLL_CYCLES;
  }

  if (serstate.aw.raw.wait_send_write && serstate.aw.raw.request_pending)
    return RAW_POLL_CYCLES;

  if (serstate.aw.raw.ack_pending) {
    if (serstate.aw.raw.ack_ready) {
      if (serstate.aw.raw.ack_wait_cycles >= RAW_ACK_QUIET_CYCLES)
        return 1;
      return RAW_ACK_QUIET_CYCLES - serstate.aw.raw.ack_wait_cycles;
    }
    return RAW_POLL_CYCLES;
  }

  return ~0U;
}

static void serialaw_senddata(u16 cmd, u8 state, const u16 *packet, size_t wcnt) {
  u32 flags = (cmd << 16) | (state << 8) | wcnt;
  u32 pkt[2 + 128] = {
    netorder32(NET_SERADWR_HEADER),  // Header magic
    netorder32(flags),               // Current device state
  };
  pack16(&pkt[2], packet, wcnt);

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

static bool serialaw_make_room(u32 client_id, u16 needed) {
  if (needed >= MAX_FPACK)
    return false;

  while (serstate.aw.peer[client_id].count + needed >= MAX_FPACK) {
    u16 offset = 0;
    if (serstate.aw.peer[client_id].recvd) {
      const u16 active_words = serstate.aw.peer[client_id].data[0] + 1;
      if (active_words >= serstate.aw.peer[client_id].count)
        return false;
      offset = active_words;
    }

    const u16 queued_words = serstate.aw.peer[client_id].data[offset] + 1;
    if (queued_words <= 1 || offset + queued_words > serstate.aw.peer[client_id].count) {
      serstate.aw.peer[client_id].count = 0;
      serstate.aw.peer[client_id].recvd = 0;
      return true;
    }

    memmove(&serstate.aw.peer[client_id].data[offset],
            &serstate.aw.peer[client_id].data[offset + queued_words],
            (serstate.aw.peer[client_id].count - offset - queued_words) * sizeof(u16));
    serstate.aw.peer[client_id].count -= queued_words;
  }

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
  else
    return (serstate.aw.peer[0].pstate == PSTATE_COMMANDS &&
            serstate.aw.peer[i].state == STATE_SYNC) ? CMD_SYNC : CMD_NONE;
}

void serialaw_master_send(void) {
  u32 i;
  u16 mvalue = read_ioreg(REG_SIOMLT_SEND);

  write_ioreg(REG_SIOMULTI0, mvalue);   // echo sent value

  if (serstate.aw.peer[0].state == STATE_SYNC) {
    if (mvalue == CMD_NONE)
      serstate.aw.peer[0].state = STATE_PACKETXG;
    else {
      // We wait for all other peers to be in SYNC state, return dummy words otherwise
      for (i = 1; i <= 3; i++)
        write_ioreg(REG_SIOMULTI0 + i, serstate.aw.peer[i].state >= STATE_SYNC ? CMD_SYNC : CMD_NONE);

      if (mvalue != CMD_SYNC && mvalue != CMD_NOP) {
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
        SRPT_DEBUG_LOG("Moving to SYNC state\n");
        serialaw_senddata(mvalue, STATE_SYNC, NULL, 0);
      }
      else if (mvalue == CMD_SYNC) {
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

  if (serstate.aw.raw.enabled)
    return raw_update(cycles);

  netpacket_poll_receive();

  serstate.aw.frcnt += cycles;

  // Packet exchange has to drain near the ROM's serial transfer rate.
  // Sync/intersync stays frame paced because it exchanges readiness/key state.
  const u32 ev_cycles = (serstate.aw.peer[netplay_client_id].state >= STATE_SYNC) ?
                        SLAVE_IRQ_CYCLES_H : SLAVE_IRQ_CYCLES_PACKET;

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
          SRPT_DEBUG_LOG("Moving to SYNC state\n");
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

    write_ioreg(REG_SIOCNT, read_ioreg(REG_SIOCNT) & ~0x80);

    return true;
  }

  return false;
}

void serialaw_net_receive(const void* buf, size_t len, uint16_t client_id) {
  const u32 *pkt = (u32*)buf;
  u16 words[4] = { 0, 0, 0, 0 };

  if (serstate.aw.raw.enabled && len == 20 && netorder32(pkt[0]) == NET_SERAWRW_HEADER) {
    const u32 kind = netorder32(pkt[1]);
    const u32 seq = netorder32(pkt[2]);
    raw_unpack_words(words, pkt);

    if (kind == RAW_KIND_REQ && netplay_client_id && client_id == 0) {
      raw_receive_request(seq, words);
    }
    else if (kind == RAW_KIND_REPLY && !netplay_client_id && client_id <= 3 &&
             serstate.aw.raw.phase == RAW_PHASE_WAIT_REPLIES &&
             seq == serstate.aw.raw.pending_seq) {
      serstate.aw.raw.reply[client_id] = words[client_id];
      serstate.aw.raw.last_send[client_id] = words[client_id];
      serstate.aw.raw.reply_mask |= (u8)(1 << client_id);
      if ((serstate.aw.raw.reply_mask & raw_expected_mask()) == raw_expected_mask())
        serstate.aw.raw.wait_cycles = 0;
      serstate.aw.raw.reply_recv_count++;
      raw_trace(RAW_TRACE_REPLY_RX, seq, words, client_id);
    }
    else if (kind == RAW_KIND_REPLY && netplay_client_id && client_id <= 3 &&
             client_id != netplay_client_id &&
             raw_client_prepare_seq(seq)) {
      serstate.aw.raw.reply[client_id] = words[client_id];
      serstate.aw.raw.last_send[client_id] = words[client_id];
      serstate.aw.raw.reply_mask |= (u8)(1 << client_id);
      serstate.aw.raw.reply_recv_count++;
      raw_trace(RAW_TRACE_REPLY_RX, seq, words, client_id);
    }
    else if (kind == RAW_KIND_BUS && netplay_client_id && client_id == 0) {
      u32 i;
      if (!serstate.aw.raw.transfer_pending && seq <= serstate.aw.raw.pending_seq) {
        serstate.aw.raw.last_drop_seq = seq;
        return;
      }
      for (i = 0; i <= 3; i++) {
        serstate.aw.raw.bus[i] = words[i];
        serstate.aw.raw.reply[i] = words[i];
        serstate.aw.raw.last_send[i] = words[i];
      }
      serstate.aw.raw.reply_mask = raw_expected_mask();
      serstate.aw.raw.pending_seq = seq;
      serstate.aw.raw.irq_pending = 1;
      serstate.aw.raw.transfer_pending = 0;
      serstate.aw.raw.wait_cycles = raw_transfer_cycles();
      serstate.aw.raw.request_pending = 0;
      serstate.aw.raw.request_wait_cycles = 0;
      serstate.aw.raw.bus_recv_count++;
      raw_trace(RAW_TRACE_BUS_RX, seq, words, read_ioreg(REG_SIOMLT_SEND));
    }
    else if (kind == RAW_KIND_ACK && !netplay_client_id && client_id <= 3 &&
             serstate.aw.raw.phase == RAW_PHASE_WAIT_ACKS &&
             seq == serstate.aw.raw.pending_seq) {
      serstate.aw.raw.ack_mask |= (u8)(1 << client_id);
      serstate.aw.raw.ack_recv_count++;
      raw_trace(RAW_TRACE_ACK_RX, seq, words, client_id);
    }
    else if (kind == RAW_KIND_REQ || kind == RAW_KIND_REPLY ||
             kind == RAW_KIND_BUS || kind == RAW_KIND_ACK) {
      serstate.aw.raw.last_drop_seq = seq;
    }
    return;
  }

  if (serstate.aw.raw.enabled)
    return;

  // MAW1 header, sanity checking.
  if (len >= 8 && netorder32(pkt[0]) == NET_SERADWR_HEADER) {
    const u32 flags = netorder32(pkt[1]);
    const u16 cmd = flags >> 16;
    const u16 ste = (flags >> 8) & 0xff;    // Peer state.
    const u16 cnt = flags & 0x00ff;         // Number of words to follow.

    serstate.aw.peer[client_id].timeout = 0;
    serstate.aw.peer[client_id].state = ste;

    SRPT_DEBUG_LOG("Got packet with state %d cmd %04x and size %d.\n", ste, cmd, cnt);

    if (ste == STATE_INTERSYNC) {
      if (cnt >= 2 && len == cnt * 2 + 8) {
        if (serialaw_make_room(client_id, cnt + 1)) {
          serstate.aw.peer[client_id].data[serstate.aw.peer[client_id].count++] = cnt;
          unpack16(&serstate.aw.peer[client_id].data[serstate.aw.peer[client_id].count], &pkt[2], cnt);
          serstate.aw.peer[client_id].count += cnt;

          SRPT_DEBUG_LOG("Received valid sync command client %d: %04x\n", client_id, cmd);
        }
      }
    }
    else if (ste == STATE_PACKETXG) {
      if (cnt >= 2 && len == cnt * 2 + 8) {
        if (serialaw_make_room(client_id, cnt + 2)) {
          // We insert this in the queue, adding a header for length.
          // Also the command value (should be 0x4FFF) is inserted too.
          serstate.aw.peer[client_id].data[serstate.aw.peer[client_id].count++] = cnt + 1;  // Packet + command
          serstate.aw.peer[client_id].data[serstate.aw.peer[client_id].count++] = cmd;
          unpack16(&serstate.aw.peer[client_id].data[serstate.aw.peer[client_id].count], &pkt[2], cnt);
          serstate.aw.peer[client_id].count += cnt;

          SRPT_DEBUG_LOG("Received valid packet from client %d (with %d words)\n",
                         client_id, cnt);
        }
        else
          SRPT_DEBUG_LOG("Packet dropped!\n");
      }
    }
  }
}

u32 serialaw_trace_value(int index) {
  const u32 local = netplay_client_id & 3;
  if (index >= 84) {
    const u32 offset = (u32)(index - 84);
    const u32 entry_offset = offset / 10;
    const u32 field = offset % 10;
    const u32 available = serstate.aw.raw.trace_count < RAW_TRACE_LEN ?
      serstate.aw.raw.trace_count : RAW_TRACE_LEN;
    u32 pos;

    if (entry_offset >= available)
      return 0;

    pos = (serstate.aw.raw.trace_pos + RAW_TRACE_LEN - 1 - entry_offset) % RAW_TRACE_LEN;
    switch (field) {
    case 0: return serstate.aw.raw.trace[pos].event;
    case 1: return serstate.aw.raw.trace[pos].seq;
    case 2: return serstate.aw.raw.trace[pos].extra;
    case 3: return serstate.aw.raw.trace[pos].words[0];
    case 4: return serstate.aw.raw.trace[pos].words[1];
    case 5: return serstate.aw.raw.trace[pos].words[2];
    case 6: return serstate.aw.raw.trace[pos].words[3];
    case 7: return serstate.aw.raw.trace[pos].siocnt;
    case 8: return serstate.aw.raw.trace[pos].rcnt;
    case 9: return serstate.aw.raw.trace[pos].flags;
    default: return 0;
    }
  }

  switch (index) {
  case 0: return serstate.aw.peer[local].state;
  case 1: return serstate.aw.peer[local].pstate;
  case 2: return serstate.aw.peer[local].count;
  case 3: return serstate.aw.peer[local].recvd;
  case 4: return serstate.aw.peer[0].state;
  case 5: return serstate.aw.peer[1].state;
  case 6: return serstate.aw.peer[2].state;
  case 7: return serstate.aw.peer[3].state;
  case 8: return serstate.aw.peer[0].count;
  case 9: return serstate.aw.peer[1].count;
  case 10: return serstate.aw.peer[2].count;
  case 11: return serstate.aw.peer[3].count;
  case 12: return read_ioreg(REG_SIOMULTI0);
  case 13: return read_ioreg(REG_SIOMULTI1);
  case 14: return read_ioreg(REG_SIOMULTI2);
  case 15: return read_ioreg(REG_SIOMULTI3);
  case 16: return read_ioreg(REG_SIOCNT);
  case 17: return serstate.aw.lastcmd;
  case 18: return serstate.aw.peer[0].pstate;
  case 19: return serstate.aw.peer[1].pstate;
  case 20: return serstate.aw.peer[2].pstate;
  case 21: return serstate.aw.peer[3].pstate;
  case 22: return serstate.aw.peer[0].data[0];
  case 23: return serstate.aw.peer[0].data[1];
  case 24: return serstate.aw.peer[0].data[2];
  case 25: return serstate.aw.peer[1].data[0];
  case 26: return serstate.aw.peer[1].data[1];
  case 27: return serstate.aw.peer[1].data[2];
  case 28: return serstate.aw.peer[2].data[0];
  case 29: return serstate.aw.peer[2].data[1];
  case 30: return serstate.aw.peer[2].data[2];
  case 31: return serstate.aw.peer[3].data[0];
  case 32: return serstate.aw.peer[3].data[1];
  case 33: return serstate.aw.peer[3].data[2];
  case 34: return serstate.aw.peer[local].data[0];
  case 35: return serstate.aw.peer[local].data[1];
  case 36: return serstate.aw.peer[local].data[2];
  case 37: return serstate.aw.peer[local].data[3];
  case 38: return serstate.aw.peer[local].data[4];
  case 39: return serstate.aw.peer[local].data[5];
  case 40: return serstate.aw.peer[local].data[6];
  case 41: return serstate.aw.peer[local].data[7];
  case 42: return serstate.aw.peer[local].data[0] & 0xff;
  case 43: return serstate.aw.raw.enabled;
  case 44: return serstate.aw.raw.phase;
  case 45: return serstate.aw.raw.reply_mask;
  case 46: return serstate.aw.raw.ack_mask;
  case 47: return serstate.aw.raw.irq_pending;
  case 48: return serstate.aw.raw.seq;
  case 49: return serstate.aw.raw.pending_seq;
  case 50: return serstate.aw.raw.last_send[0];
  case 51: return serstate.aw.raw.last_send[1];
  case 52: return serstate.aw.raw.last_send[2];
  case 53: return serstate.aw.raw.last_send[3];
  case 54: return serstate.aw.raw.bus[0];
  case 55: return serstate.aw.raw.bus[1];
  case 56: return serstate.aw.raw.bus[2];
  case 57: return serstate.aw.raw.bus[3];
  case 58: return read_ioreg(REG_SIOMLT_SEND);
  case 59: return serstate.aw.raw.pending_host_send;
  case 60: return serstate.aw.raw.wait_cycles;
  case 61: return serstate.aw.raw.pending_start;
  case 62: return serstate.aw.raw.timeout_count;
  case 63: return serstate.aw.raw.request_pending;
  case 64: return serstate.aw.raw.request_seq;
  case 65: return serstate.aw.raw.wait_send_write;
  case 66: return serstate.aw.raw.request_wait_cycles;
  case 67: return serstate.aw.raw.send_write_count;
  case 68: return serstate.aw.raw.fresh_send_count;
  case 69: return serstate.aw.raw.stale_reply_count;
  case 70: return serstate.aw.raw.ack_send_count;
  case 71: return serstate.aw.raw.ack_recv_count;
  case 72: return serstate.aw.raw.reply_send_count;
  case 73: return serstate.aw.raw.reply_recv_count;
  case 74: return serstate.aw.raw.bus_recv_count;
  case 75: return serstate.aw.raw.last_ack_send_seq;
  case 76: return serstate.aw.raw.last_drop_seq;
  case 77: return read_ioreg(REG_RCNT);
  case 78: return serstate.aw.raw.transfer_pending;
  case 79: return serstate.aw.raw.ack_pending;
  case 80: return serstate.aw.raw.ack_ready;
  case 81: return serstate.aw.raw.ack_wait_cycles;
  case 82: return raw_transfer_cycles();
  case 83: return serstate.aw.raw.fast_ack;
  default: return 0;
  }
}

void serialaw_set_raw_bus_enabled(int enabled) {
  const u8 next_enabled = enabled ? 1 : 0;
  if (serstate.aw.raw.enabled != next_enabled) {
    const u8 fast_ack = serstate.aw.raw.fast_ack;
    memset(&serstate.aw.raw, 0, sizeof(serstate.aw.raw));
    serstate.aw.raw.enabled = next_enabled;
    serstate.aw.raw.fast_ack = fast_ack;
  }
}

void serialaw_set_raw_fast_ack_enabled(int enabled) {
  serstate.aw.raw.fast_ack = enabled ? 1 : 0;
}
