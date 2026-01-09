

import './Grid.css';
import LaserScanPlot from "./LaserScanPlot"; 
import CameraFeed from "./CameraFeed";
import LaserScanFan from "./LaserScanFan";
import Alerts from "./Alerts";
import SimControls from './SimControls';
import ArtificialHorizonCanvas from './ArtificialHorizonCanvas';
import DepthAltimeter from './DepthAltimeter';

export default function Grid(){
    
return (
    <>  
        <Alerts />
        <div className="wrapper">
            {/* Top left panel */}
            <div className="top-left-grid">
                {/* Upper half */}
                {/* <div> <img src="https://i.imgur.com/iZKjJI4.png" alt="" /> </div> */}
                <div><ArtificialHorizonCanvas /></div>

                {/* Lower half */}
                <div className="temp">
                    <div><SimControls /></div>
                    <div><img src="https://user-images.githubusercontent.com/33184844/207543214-2cd65dfa-8473-4bc8-94ac-f6c2d934cd48.png" alt="" /></div>
                    <div><DepthAltimeter /></div>

                </div>
            </div>

            {/* Top right panel */}
            <div><LaserScanFan /></div>

            {/* Bottom left panel */}
            <div><LaserScanPlot /></div>

            {/* Bottom right panel */}
            <div><CameraFeed /></div>

        </div>
    </>
);
}

