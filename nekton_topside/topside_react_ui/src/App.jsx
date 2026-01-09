import './App.css'
import { RosProvider } from './context/RosProvider';
import Grid from './components/Grid'

function App() {

  return (
    <>
      <RosProvider url="ws://localhost:9090">
        <Grid />
      </RosProvider>
    </>
  )
}

export default App
