import { useCallback, useEffect, useRef, useState } from 'react'

const MAX_EVENTS = 50
const MAX_HISTORY_POINTS = 60
const RECONNECT_DELAY_MS = 2000

/**
 * 백엔드(main.py)의 /ws 에 붙어서 차량 상태/이벤트를 받고, 명령을 보내는 훅.
 * host가 바뀌면 기존 연결을 끊고 새로 연결한다.
 */
export function useDashboardSocket(host) {
  const [cars, setCars] = useState({ A: null, B: null, C: null })
  const [events, setEvents] = useState([])
  const [distanceHistory, setDistanceHistory] = useState([])
  const [connected, setConnected] = useState(false)

  const wsRef = useRef(null)
  const reconnectTimerRef = useRef(null)

  useEffect(() => {
    if (!host) {
      setConnected(false)
      return
    }

    let cancelled = false

    const connect = () => {
      if (cancelled) return

      const ws = new WebSocket(`ws://${host}:8000/ws`)
      wsRef.current = ws

      ws.onopen = () => setConnected(true)

      ws.onmessage = (event) => {
        let payload
        try {
          payload = JSON.parse(event.data)
        } catch {
          return
        }

        if (payload.type === 'state') {
          setCars(payload.cars)
          setDistanceHistory((prev) => {
            const point = {
              t: Date.now(),
              B: payload.cars?.B?.distance_to_front ?? null,
              C: payload.cars?.C?.distance_to_front ?? null,
            }
            const next = [...prev, point]
            return next.length > MAX_HISTORY_POINTS ? next.slice(-MAX_HISTORY_POINTS) : next
          })
        } else if (payload.type === 'event') {
          setEvents((prev) => {
            const next = [{ ...payload.event, _receivedAt: Date.now() }, ...prev]
            return next.length > MAX_EVENTS ? next.slice(0, MAX_EVENTS) : next
          })
        }
      }

      ws.onclose = () => {
        setConnected(false)
        if (!cancelled) {
          reconnectTimerRef.current = setTimeout(connect, RECONNECT_DELAY_MS)
        }
      }

      ws.onerror = () => {
        ws.close()
      }
    }

    connect()

    return () => {
      cancelled = true
      clearTimeout(reconnectTimerRef.current)
      wsRef.current?.close()
    }
  }, [host])

  const sendCommand = useCallback((command, target = 'all') => {
    const ws = wsRef.current
    if (ws && ws.readyState === WebSocket.OPEN) {
      ws.send(JSON.stringify({ command, target }))
    }
  }, [])

  return { cars, events, distanceHistory, connected, sendCommand }
}
