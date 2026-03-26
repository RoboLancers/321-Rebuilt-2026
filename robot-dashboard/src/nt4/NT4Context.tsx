import { createContext, ReactNode, useEffect, useRef, useState } from 'react'
import { NT4Client } from './NT4Client'
import { SimClient } from './SimClient'
import { ROBOT_ADDRESS, SIM_MODE } from '../config'

// Both NT4Client and SimClient share the same subscribe/connect/disconnect
// interface, so we type the context as the union.
type AnyClient = NT4Client | SimClient

export const NT4ClientContext = createContext<AnyClient | null>(null)
export const NT4ConnectedContext = createContext(false)

export function NT4Provider({ children }: { children: ReactNode }) {
  const [connected, setConnected] = useState(false)

  const clientRef = useRef<AnyClient | null>(null)
  if (!clientRef.current) {
    clientRef.current = SIM_MODE
      ? new SimClient(ROBOT_ADDRESS, setConnected)
      : new NT4Client(ROBOT_ADDRESS, setConnected)
  }

  useEffect(() => {
    const client = clientRef.current!
    client.connect()
    return () => client.disconnect()
  }, [])

  return (
    <NT4ClientContext.Provider value={clientRef.current}>
      <NT4ConnectedContext.Provider value={connected}>
        {children}
      </NT4ConnectedContext.Provider>
    </NT4ClientContext.Provider>
  )
}
