import { useEffect, useState } from 'react'
import { useDashboardSocket } from './useDashboardSocket'
import { CarCard } from './components/CarCard'
import { DistanceChart } from './components/DistanceChart'
import { EventLog } from './components/EventLog'
import { ControlBar } from './components/ControlBar'
import './App.css'

const CAR_IDS = ['A', 'B', 'C']
const HOST_STORAGE_KEY = 'rcteam3_backend_host'

function App() {
  const [host, setHost] = useState(() => localStorage.getItem(HOST_STORAGE_KEY) ?? '')
  const [hostInput, setHostInput] = useState(host)

  useEffect(() => {
    if (host) localStorage.setItem(HOST_STORAGE_KEY, host)
  }, [host])

  const { cars, events, distanceHistory, connected, sendCommand } = useDashboardSocket(host)

  const handleHostSubmit = (e) => {
    e.preventDefault()
    setHost(hostInput.trim())
  }

  return (
    <div className="dashboard">
      <header className="dashboard__header">
        <h1>RC Team 3 관제 대시보드</h1>
        <form className="host-form" onSubmit={handleHostSubmit}>
          <label htmlFor="host-input">백엔드 서버 주소</label>
          <input
            id="host-input"
            value={hostInput}
            onChange={(e) => setHostInput(e.target.value)}
            placeholder="예: 172.20.10.5"
          />
          <button type="submit">연결</button>
          <span className={`status-dot ${connected ? 'status-dot--good' : 'status-dot--critical'}`} />
          <span className="host-form__status">{connected ? '연결됨' : '연결 안 됨'}</span>
        </form>
      </header>

      <ControlBar onCommand={sendCommand} />

      <section className="dashboard__cars">
        {CAR_IDS.map((carId) => (
          <CarCard key={carId} carId={carId} data={cars[carId]} onCommand={sendCommand} />
        ))}
      </section>

      <section className="dashboard__lower">
        <DistanceChart history={distanceHistory} />
        <EventLog events={events} />
      </section>
    </div>
  )
}

export default App
