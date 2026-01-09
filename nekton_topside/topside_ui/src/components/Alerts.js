export default class Alerts extends HTMLElement {
  constructor() {
    super();
    this.shadow = this.attachShadow({mode: "open"});
  }
  connectedCallback() {
    const css = document.createElement("style");
    const wrapper = document.createElement("span");
    wrapper.innerHTML = `
    <div class="alertBox">
      <div class="alertBoxGroup">
        <div id="overheated" class="alert">Overheated</div>
        <div id="bell" class="alert">Bell</div>
        <div id="lowBattery" class="alert">Low Battery</div>
        <div id="leakSensor" class="alert">Leak Sensor</div>
        <div id="alarm" class="alert">Alarm</div>
        <button id="testButton" type="button">Test Button</button>
      </div>
    </div>
    `;
    css.textContent = `
    .alertBoxGroup {
      display: flex;
      flex-wrap: wrap;
      justify-content: center;
      align-items: center;
      padding: 10px;
    }
    .alert {
      text-align: center;
      height: 50px;
      width: 150px;
      margin: 10px;
      padding: 5px;
      background-color: green;
      border-radius: 15px;
      color: white;
      font-weight: bold;
      line-height: 50px;
    }
    button {
      height: 50px;
      margin: 10px;
      padding: 0 20px;
    }
    `;
    this.shadow.appendChild(css);
    this.shadow.appendChild(wrapper);

    // Add event listener for the test button
    this.shadow.getElementById('testButton').addEventListener('click', () => {
      this.testButton();
    });
  }

  static get observedAttributes() {
    return ["overheated", "bell", "lowBattery", "leakSensor", "alarm"];
  }

  // Function to handle button click
  testButton() {
    const exampleMessage = {
      overheated: true,
      bell: false,
      lowBattery: true,
      leakSensor: false,
      alarm: true
    };
    this.updateAlerts(exampleMessage);
    console.log('Test Button Function Finished');
  }

  attributeChangedCallback(attrName, newVal) {
    const alertElement = this.shadow.querySelector(`#${attrName}`);
    if (alertElement) {
      alertElement.style.backgroundColor = newVal === "true" ? "red" : "green";
    }
  }

  updateAlerts(message) {
    try {
      Object.keys(message).forEach(alertType => {
        const alertElement = this.shadow.querySelector(`#${alertType}`);
        if (alertElement) {
          alertElement.style.backgroundColor = message[alertType] ? 'red' : 'green';
        }
      });
    } catch (error) {
      console.error('Error parsing alert data:', error);
    }
  }
}

customElements.define("nekton-alert", Alerts);