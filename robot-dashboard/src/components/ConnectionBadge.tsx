import { useConnected } from '../hooks/useNT4'
import { ROBOT_ADDRESS, ROBOT_MDNS } from '../config'

export function ConnectionBadge() {
  const connected = useConnected()
  return (
    <div className={`connection-badge ${connected ? 'connected' : 'disconnected'}`}>
      <span className="connection-dot" />
      {connected
        ? `NT4 · ${ROBOT_ADDRESS}`
        : `Connecting to ${ROBOT_MDNS}…`}
    </div>
  )
}
