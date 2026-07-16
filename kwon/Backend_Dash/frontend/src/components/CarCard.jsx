const MODE_LABEL = {
  idle: '대기',
  driving: '주행 중',
  stopped: '정지',
  emergency_stop: '비상정지',
  offline: '연결 끊김',
}

function statusClassFor(mode) {
  if (mode === 'offline') return 'status-dot--critical'
  if (mode === 'emergency_stop') return 'status-dot--critical'
  if (mode === 'driving') return 'status-dot--good'
  return 'status-dot--warning'
}

export function CarCard({ carId, data, onCommand }) {
  const mode = data?.mode ?? 'offline'
  const isOffline = !data || mode === 'offline'

  return (
    <div className="car-card">
      <div className="car-card__header">
        <span className={`status-dot ${statusClassFor(mode)}`} />
        <h3>{carId} 차량</h3>
        <span className="car-card__mode">{MODE_LABEL[mode] ?? mode}</span>
      </div>

      <dl className="car-card__stats">
        <div>
          <dt>속도</dt>
          <dd>{isOffline ? '—' : data.speed ?? 0}</dd>
        </div>
        <div>
          <dt>조향각</dt>
          <dd>{isOffline ? '—' : `${data.steering_angle ?? 0}°`}</dd>
        </div>
        <div>
          <dt>전방 차간거리</dt>
          <dd>{isOffline || data.distance_to_front == null ? '—' : `${data.distance_to_front} cm`}</dd>
        </div>
        <div>
          <dt>브레이크</dt>
          <dd>{isOffline ? '—' : data.brake ? 'ON' : 'OFF'}</dd>
        </div>
      </dl>

      <div className="car-card__actions">
        <button type="button" onClick={() => onCommand('start', carId)}>
          시작
        </button>
        <button type="button" onClick={() => onCommand('stop', carId)}>
          정지
        </button>
      </div>
    </div>
  )
}
