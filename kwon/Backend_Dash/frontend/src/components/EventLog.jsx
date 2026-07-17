const EVENT_LABEL = {
  brake: '급제동',
  lane_change: '차선 변경',
  emergency_stop: '비상 정지',
  obstacle_detected: '장애물 감지',
}

const EVENT_SEVERITY = {
  emergency_stop: 'critical',
  brake: 'serious',
  obstacle_detected: 'warning',
  lane_change: 'good',
}

function formatDetail(eventType, detail = {}) {
  if (eventType === 'lane_change') {
    if (detail.direction === 'left') return '← 왼쪽으로 변경'
    if (detail.direction === 'right') return '오른쪽으로 변경 →'
    return ''
  }
  if (eventType === 'obstacle_detected') {
    const confidence = Math.round((detail.confidence ?? 0) * 100)
    return `${detail.object_type ?? '?'} (${confidence}%)`
  }
  if (eventType === 'emergency_stop') return detail.reason ?? ''
  return ''
}

export function EventLog({ events }) {
  return (
    <div className="event-log">
      <h3>시스템 로그</h3>
      <ul>
        {events.length === 0 && <li className="event-log__empty">아직 수신된 이벤트가 없습니다</li>}
        {events.map((e, i) => {
          const ts = e.timestamp ? e.timestamp * 1000 : e._receivedAt
          return (
            <li key={`${e._receivedAt}-${i}`}>
              <span className={`status-dot status-dot--${EVENT_SEVERITY[e.event_type] ?? 'warning'}`} />
              <span className="event-log__time">
                {new Date(ts).toLocaleTimeString('ko-KR', { hour12: false })}
              </span>
              <span className="event-log__car">{e.car_id}</span>
              <span className="event-log__type">{EVENT_LABEL[e.event_type] ?? e.event_type}</span>
              <span className="event-log__detail">{formatDetail(e.event_type, e.detail)}</span>
            </li>
          )
        })}
      </ul>
    </div>
  )
}
