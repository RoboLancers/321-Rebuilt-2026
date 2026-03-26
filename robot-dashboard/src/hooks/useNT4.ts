import { useContext, useEffect, useState } from 'react'
import { NT4ClientContext, NT4ConnectedContext } from '../nt4/NT4Context'
// NT4ClientContext holds either a real NT4Client or SimClient — both expose .subscribe()

export function useConnected(): boolean {
  return useContext(NT4ConnectedContext)
}

export function useNT4Number(topic: string, defaultValue = 0): number {
  const client = useContext(NT4ClientContext)
  const [value, setValue] = useState(defaultValue)

  useEffect(() => {
    if (!client) return
    return client.subscribe(topic, (v) => {
      if (typeof v === 'number') setValue(v)
    })
  }, [client, topic])

  return value
}

export function useNT4Boolean(topic: string, defaultValue = false): boolean {
  const client = useContext(NT4ClientContext)
  const [value, setValue] = useState(defaultValue)

  useEffect(() => {
    if (!client) return
    return client.subscribe(topic, (v) => {
      if (typeof v === 'boolean') setValue(v)
    })
  }, [client, topic])

  return value
}

export function useNT4String(topic: string, defaultValue = ''): string {
  const client = useContext(NT4ClientContext)
  const [value, setValue] = useState(defaultValue)

  useEffect(() => {
    if (!client) return
    return client.subscribe(topic, (v) => {
      if (typeof v === 'string') setValue(v)
    })
  }, [client, topic])

  return value
}
