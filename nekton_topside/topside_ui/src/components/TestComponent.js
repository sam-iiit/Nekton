/**
 * An Example component that implements basic functionality such as styling and attributes
 * If you are familiar with react, this is formatted almost like a component
 * 
 * When you are done with the component, go to ./nekton-component.js and add
 * import {Your_Class_Name} from './{Your_File_Name}.js';
 * For this test component: import TestComp from './TestComponent.js';
 */
export default class TestComp extends HTMLElement { 
  //DO NOT TOUCH THIS LOGIC. This is to initialize the HTMLElement Constructor
    constructor() {
      super();
      this.shadow = this.attachShadow({mode: "open"});
    }
    /**
     * This connects everything to our blank HTMLComponent
     */
    connectedCallback() {
        const css = document.createElement("style");
        //This holds the HTML for your component. Every component created needs a wrapper
        const wrapper = document.createElement("span");
        //This is where your HTML goes
        wrapper.innerHTML = `
        <div class="box">
        <p id="content"></p>
        </div>
        `

        /**
         * This is the component's styling logic. Think of it like a CSS file.
         */
        css.textContent = `
        .box {
          text-align: center;
          width: fit-content;
          margin-left: 10px;
          margin-top: 10px;
          padding: 5px;
          background-color: white;
          border-radius: 15px;
        }
        `
        /**
         * Attaches everything to the DOM.
         * IMPORTANT NODE: instead of document (function)
         * use this.shadow. This is because the component IS NOT in a regular DOM.
         * Instead, it is a shadow DOM. For details on what this is,
         * see: https://developer.mozilla.org/en-US/docs/Web/API/Web_components/Using_shadow_DOM/shadowdom.svg
         */
        this.shadow.appendChild(css);
        this.shadow.appendChild(wrapper);
        this.shadow.getElementById("content").textContent = this.getAttribute("text");
    }
  /**
   * This is how attributes are passed into the component.
   * For example, If i have 
   * 
   */
    static get observedAttributes() {
      return ["text"];
    }
  }
/**
 * First Arg: Name of your component. This is what you type when you want to initialize it.
 * Ex: I have a SimController and that 
 * Second Arg: The Class defined above
 * 
 */
customElements.define("test-comp", TestComp);  
