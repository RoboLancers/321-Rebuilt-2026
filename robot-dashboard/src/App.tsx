import { NT4Provider } from './nt4/NT4Context'
import { ConnectionBadge } from './components/ConnectionBadge'
import { SideView } from './components/SideView'
import { FrontView } from './components/FrontView'
import { StatusPanel } from './components/StatusPanel'

export default function App() {
  return (
    <NT4Provider>
      <div className="app">
        <header className="app-header">
          <span className="app-title">321 · RoboLancers</span>
          <ConnectionBadge />
        </header>

        <main className="app-body">
          <div className="views-row">
            <SideView />
            <FrontView />
          </div>
          <StatusPanel />
        </main>
      </div>
    </NT4Provider>
  )
}
