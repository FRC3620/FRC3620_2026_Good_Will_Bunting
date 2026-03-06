import { useState, useEffect } from 'react'
import { initNT, sendHello } from "./nt"
import reactLogo from './assets/react.svg'
import viteLogo from '/vite.svg'
import './App.css'

function App() {
  const [count, setCount] = useState(0)

  useEffect(() => {
    initNT()
  }, [])

  const handleClick = () => {
    const newCount = count + 1
    setCount(newCount)
    sendHello(`Hello ${newCount}`)
  }
}

export default App
