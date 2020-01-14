// Abstract robot class : exposes interface for dynamical updates
// NOTE: our optimal safety controller can only handle control-affine systems
class Robot extends PIXI.Sprite {
  // Update function that realizes the dynamical state equations
  dynamicUpdate(delT,u){
    for(let curDim = 0; curDim < this.states.length; curDim++){
      this.states[curDim] += this.dynamics(u,curDim) * delT;
    }
  }
  // Function that passes out the dynamical update for the current state
  dynamics(u,stateNum){
    let result = this.driftDynamics()[stateNum];
    // Mix in the control affine term
    for(let uNum = 0;uNum < u.length;uNum++){
      result += this.controlCoefficient()[uNum][stateNum] * u[uNum]
    }
    return result;
  }
  driftDynamics(){
    return(
      [
        0,
        0,
        0,
      ]
    );
  }
  // Coefficient matrix for the control input in the dynamical equations
  // Realized as a function to include possible state dependence
  controlCoefficient(){
    return [0];
  }
  // Method that translates states from the state vector into the corresponding
  // position for rendering in the JS
  displayState(){
    this.x = this.states[0];
    this.y = this.states[1];
    this.rotation = this.states[2];
  }
  // Constructor initializes PIXI.Sprite members and sets initial state
  constructor(texture){
    // Image
    super(texture);
    this.pivot.x = 50 ; this.pivot.y = 50;
    this.width = 100 ; this.height = 100;
    this.spinout = 0;
    // State Vector
    this.states = [0,0,0];
    // Display State
    this.displayState();
  }
  // Method to be called each loop
  update(delT,u){
    this.dynamicUpdate(delT,u);
    this.displayState();
  }
}

// Double Integrating simplified Quadrotor
class QuadrotorRobot extends Robot {
  // Drift dynamics function for the dynamical equations
  driftDynamics(){
    return(
      [
        this.states[1],
        0,
        this.states[3],
        0,
      ]
    );
  }
  // Coefficient matrix for the control input in the dynamical equations
  controlCoefficient(){
    return(
      [[0,1,0,0],
      [0,0,0,1]]
    );
  }
  // Method that translates states from the state vector into the corresponding
  // position for rendering in the JS
  displayState(){
    let mappedState =
      graphics.mapper.mapStateToPosition(this.states[0],this.states[2]);
    this.x =  mappedState[0];
    this.y =  mappedState[1];
    //this.rotation = this.states[1]/15.0
    this.rotation = 0;
  }
  // Constructor initializes PIXI.Sprite members fitting the quadrotor Texture
  // and sets initial state data
  constructor(initial_state){
    // Image
    super(PIXI.Texture.fromImage("QuadcopterSide.png"));
    this.pivot.x = 100 ; this.pivot.y = 50;
    this.width = 100 ; this.height = 100;
    // State Vector
    this.states = initial_state;
    // Display State
    this.displayState();
  }
}

// Double Integrating simplified Quadrotor
class VerticalQuadrotorRobot extends Robot {
  // Drift dynamics function for the dynamical equations
  driftDynamics(){
    return(
      [
        this.states[1],
        0,
      ]
    );
  }
  // Coefficient matrix for the control input in the dynamical equations
  controlCoefficient(){
    return [[0,1]];
  }
  // Method that translates states from the state vector into the corresponding
  // position for rendering in the JS
  displayState(){
    let mappedState = graphics.mapper.mapStateToPosition(0,this.states[0]);
    this.x =  mappedState[0];
    this.y =  mappedState[1];
    //this.rotation = this.states[1]/15.0
    this.rotation = 0;
  }
  // Constructor initializes PIXI.Sprite members fitting the quadrotor Texture
  // and sets initial state data
  constructor(initial_state){
    // Image
    super(PIXI.Texture.fromImage("QuadcopterSide.png"));
    this.pivot.x = 100 ; this.pivot.y = 50;
    this.width = 100 ; this.height = 50;
    // State Vector
    this.states = initial_state;
    // Display State
    this.displayState();
  }
}

// Double Integrating simplified Quadrotor
class DubinsRobot extends Robot {
  // Dynamical update function that realizes the double integrator Diff. Eq.
  dynamicUpdate(delT,u){
    for(let curDim = 0; curDim < this.states.length; curDim++){
      this.states[curDim] += this.dynamics(u,curDim) * delT;
    }
    // Wrap around the angle
    if(this.states[2] > Math.PI){
      this.states[2] -= 2*Math.PI;
      console.log("wrapped 2pi down");
    }
    if(this.states[2] < -Math.PI){
      this.states[2] += 2*Math.PI;
      console.log("wrapped 2pi up");
    }
  }
  // Function that passes out the dynamical update for the current state
  dynamics(u,stateNum){
    let result = this.driftDynamics()[stateNum];
    // Mix in the control affine term
    for(let uNum = 0;uNum < u.length;uNum++){
      result += this.controlCoefficient()[uNum][stateNum] * u[uNum]
    }
    return result;
  }
  // Drift dynamics function for the dynamical equations
  driftDynamics(){
    return(
      [
        this.speed * Math.cos(this.states[2]),
        this.speed * Math.sin(this.states[2]),
        0,
      ]
    );
  }
  // Coefficient matrix for the control input in the dynamical equations
  controlCoefficient(){
    return [[0,0,1]];
  }
  // Method that translates states from the state vector into the corresponding
  // position for rendering in the JS
  displayState(){
    let mappedState =
        graphics.mapper.mapStateToPosition(this.states[0],this.states[1]);
    this.x =  mappedState[0];
    this.y =  mappedState[1];
    if(this.spinout > 0){
      this.rotation += 6.2830/100;
      this.spinout -= 1;
    }
    else{
      this.rotation = this.states[2] + 3.1415/2;
    }
  }
  // Constructor initializes PIXI.Sprite members fitting the quadrotor Texture
  // and sets initial state data
  constructor(initial_state,velocity,tint){
    // Image
    super(PIXI.Texture.fromImage("DubinsCarV2.png"));
    this.width = 1.1 * graphics.mapper.Mxx;
    this.height = 1.1 * graphics.mapper.Mxx;
//    this.width = 80 * 3/7; this.height = 80 * 3/7;
    this.pivot.x = 100 ; this.pivot.y = 100;
    this.tint = tint;
    // State Vector
    this.states = initial_state;
    this.speed = velocity;
    // Display State
    this.displayState();
  }
}
