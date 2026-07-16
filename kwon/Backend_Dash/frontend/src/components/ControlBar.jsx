export function ControlBar({ onCommand }) {
  return (
    <div className="control-bar">
      <button type="button" className="control-bar__btn control-bar__btn--good" onClick={() => onCommand('start', 'all')}>
        전체 시작
      </button>
      <button type="button" className="control-bar__btn" onClick={() => onCommand('stop', 'all')}>
        전체 정지
      </button>
      <button
        type="button"
        className="control-bar__btn control-bar__btn--critical"
        onClick={() => onCommand('emergency_stop', 'all')}
      >
        🚨 긴급 정지
      </button>
    </div>
  )
}
