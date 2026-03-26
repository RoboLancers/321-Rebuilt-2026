import { decode } from '@msgpack/msgpack'

// Supported NT4 value types
export type NT4Value = boolean | number | string | boolean[] | number[] | string[] | Uint8Array

type ValueCallback = (value: NT4Value, timestampUs: number) => void

interface TextMessage {
  method: string
  params: Record<string, unknown>
}

/**
 * Minimal NT4 WebSocket client.
 * Connects to the robot at ws://<address>:5810/nt/robot-dashboard
 * and automatically reconnects on disconnect.
 *
 * Binary frames are decoded as MessagePack arrays: [topicId, timestampUs, typeId, value]
 * Text frames are JSON arrays of control messages (announce / unannounce).
 */
export class NT4Client {
  private ws: WebSocket | null = null
  private subsByTopic = new Map<string, Map<number, ValueCallback>>()
  private topicIdToName = new Map<number, string>()
  private nextSubUid = 1
  private nextCbId = 1
  private reconnectHandle: ReturnType<typeof setTimeout> | null = null

  constructor(
    private readonly address: string,
    private readonly onConnectionChange: (connected: boolean) => void,
  ) {}

  connect() {
    try {
      this.ws = new WebSocket(
        `ws://${this.address}:5810/nt/robot-dashboard`,
        'networktables.first.wpi.edu',
      )
      this.ws.binaryType = 'arraybuffer'
      this.ws.onopen = this.handleOpen
      this.ws.onclose = this.handleClose
      this.ws.onerror = this.handleError
      this.ws.onmessage = this.handleMessage
    } catch {
      this.scheduleReconnect()
    }
  }

  disconnect() {
    if (this.reconnectHandle) clearTimeout(this.reconnectHandle)
    this.ws?.close()
    this.ws = null
  }

  /**
   * Subscribe to a NetworkTables topic.
   * Returns an unsubscribe function — call it to stop receiving updates.
   */
  subscribe(topic: string, cb: ValueCallback): () => void {
    const cbId = this.nextCbId++

    if (!this.subsByTopic.has(topic)) {
      this.subsByTopic.set(topic, new Map())
      // Only send the subscribe message if already connected; handleOpen will
      // replay all subscriptions on reconnect.
      if (this.ws?.readyState === WebSocket.OPEN) {
        this.sendSubscribe(topic)
      }
    }
    this.subsByTopic.get(topic)!.set(cbId, cb)

    return () => {
      const map = this.subsByTopic.get(topic)
      if (!map) return
      map.delete(cbId)
      // We intentionally leave the NT subscription alive to avoid churn.
    }
  }

  // ── WebSocket handlers ─────────────────────────────────────────────────────

  private handleOpen = () => {
    this.onConnectionChange(true)
    // Replay all active subscriptions after reconnect.
    for (const topic of this.subsByTopic.keys()) {
      this.sendSubscribe(topic)
    }
  }

  private handleClose = () => {
    this.onConnectionChange(false)
    this.topicIdToName.clear()
    this.scheduleReconnect()
  }

  private handleError = () => {
    // onclose fires right after onerror; reconnect logic lives there.
    this.ws?.close()
  }

  private handleMessage = (event: MessageEvent) => {
    if (typeof event.data === 'string') {
      this.handleTextFrame(event.data)
    } else {
      this.handleBinaryFrame(event.data as ArrayBuffer)
    }
  }

  // ── Frame handling ─────────────────────────────────────────────────────────

  private handleTextFrame(raw: string) {
    let messages: TextMessage[]
    try {
      messages = JSON.parse(raw) as TextMessage[]
    } catch {
      return
    }

    for (const msg of messages) {
      if (msg.method === 'announce') {
        const { name, id } = msg.params as { name: string; id: number }
        this.topicIdToName.set(id, name)
      } else if (msg.method === 'unannounce') {
        const { id } = msg.params as { id: number }
        this.topicIdToName.delete(id)
      }
    }
  }

  private handleBinaryFrame(buf: ArrayBuffer) {
    // NT4 binary message: MessagePack array [topicId, timestampUs, typeId, value]
    let msg: unknown
    try {
      msg = decode(new Uint8Array(buf))
    } catch {
      return
    }
    if (!Array.isArray(msg) || msg.length < 4) return

    const [topicId, timestamp, , value] = msg as [number, number, number, NT4Value]
    const name = this.topicIdToName.get(topicId)
    if (!name) return

    const callbacks = this.subsByTopic.get(name)
    if (!callbacks) return
    for (const cb of callbacks.values()) {
      cb(value, timestamp)
    }
  }

  // ── Helpers ────────────────────────────────────────────────────────────────

  private sendSubscribe(topic: string) {
    if (this.ws?.readyState !== WebSocket.OPEN) return
    this.ws.send(
      JSON.stringify([
        {
          method: 'subscribe',
          params: {
            topics: [topic],
            subuid: this.nextSubUid++,
            options: {
              periodic: 0.05, // 50 ms update rate
              all: false,
              topicsonly: false,
              prefix: false,
            },
          },
        },
      ]),
    )
  }

  private scheduleReconnect() {
    this.reconnectHandle = setTimeout(() => this.connect(), 3000)
  }
}
