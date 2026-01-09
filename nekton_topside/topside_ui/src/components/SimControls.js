export default class SimControls extends HTMLElement { 
    constructor() {
        super();
        this.ros = ros;
        this.attachShadow({ mode: "open" });
        this.pausePhysicsService = new ROSLIB.Service({
            ros: ros,
            name: "/pause_physics",
            serviceType: "std_srvs/srv/Empty",
          });
          
        this.unpausePhysicsService = new ROSLIB.Service({
            ros: ros,
            name: "/unpause_physics",
            serviceType: "std_srvs/srv/Empty",
          });
    }

    connectedCallback() {
        const css = document.createElement("style");
        const wrapper = document.createElement("span");
        wrapper.innerHTML = `
        <div class="box">
          <h1>Nekton GUI roslib Example</h1>
          <p>Check your web console for an output.</p>
          <hr />
          <div class="btn-group">
            <button id="pause_physics">Pause Physics</button>
            <button id="unpause_physics">Unpause Physics</button>
          </div>
        </div>`;

        css.textContent = `
        .box {
          text-align: center;
          width: fit-content;
          margin-left: 10px;
          margin-top: 10px;
          padding: 5px;
          background-color: white;
          border-radius: 15px;
        }`;

        this.shadowRoot.appendChild(css);
        this.shadowRoot.appendChild(wrapper);

        // Fix event listener
        this.shadowRoot.getElementById('pause_physics').addEventListener('click', () => {
            this.pausePhysics();
        });

        this.shadowRoot.getElementById('unpause_physics').addEventListener('click', () => {
            this.unpausePhysics();
        });

        // Ensure the 'output' element exists before using it
        const outputElement = document.getElementById("output");
        if (outputElement) {
            outputElement.innerHTML = myVariable; 
        } else {
            console.warn("'output' element not found in the main document.");
        }
    }
    

    pausePhysics() {
        if (this.pausePhysicsService) {
            this.pausePhysicsService.callService(new ROSLIB.ServiceRequest({}));
            console.log('Physics is now paused.');
        } else {
            console.error("pausePhysicsService is not defined.");
        }
    }

    unpausePhysics() {
        if (this.unpausePhysicsService) {
            this.unpausePhysicsService.callService(new ROSLIB.ServiceRequest({}));
            console.log('Physics is now unpaused.');
        } else {
            console.error("unpausePhysicsService is not defined.");
        }
    }
}

customElements.define("sim-controls", SimControls);
