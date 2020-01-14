// Controller 'virtual' class
class Controller {
  constructor(_robot){
    this.robot = _robot;
  }
  // Returns the current control value responding to the robot's state
  u(){
    return [0];
  }
}

// Controller class that takes in a list of small 1D controllers, and
// concatenates them into one multi-dimensional control output
class Concat_Contr extends Controller {
  // Pass in an array of 1D controller objects
  constructor(_robot,_controllerArray){
    super(_robot);
    this.robot = _robot;
    this.controllers = _controllerArray;
  }
  // Method for updating the setpoint to be tracked
  updateSetpoint(_set){
    for(let curU = 0; curU < this.controllers.length; curU++){
      // HACK: Hardcodes which setpoints are governed by which controller
      // Should make a generalized index or something that will send these
      // setpoints to update the correct controller's setpoint
      (this.controllers[curU]).updateSetpoint(_set[curU])
    }
  }
  // Returns the current control value responding to the robot's state
  u(){
    let uResultant = [];
    for(let curU = 0; curU < this.controllers.length; curU++){
      uResultant[curU] = (this.controllers[curU]).u()[0]
    }
    return uResultant;
  }
}

// PD Controller class
class PD_Contr extends Controller {
  constructor(_robot,_set,_controlledState){
    super(_robot);
    this.setpoint = _set;
    // PID Gains
    this.K_P = -2;
    this.K_D = -2;
    // Which indexed state to reference errors from
    this.controlledState = _controlledState;
    // Memory variables
    this.lastU = 0;
  }
  // Method for updating the setpoint to be tracked
  updateSetpoint(_set){
    this.setpoint = _set;
  }
  // Returns the current control value responding to the robot's state
  u(){
    // Calculate the components of the PD Controller
    let P = this.K_P * (this.robot.states[this.controlledState] - this.setpoint);
    let D = this.K_D * (this.robot.dynamics(this.lastU,this.controlledState));
    let resultU = P + D;
    // Store and send the resultant control
    this.lastU = resultU;
    return [resultU];
  }
}

// Controller that does nothing
class Zero_Contr extends Controller {
  constructor(_robot){
    super(_robot);
  }
  // Returns the current control value responding to the robot's state
  u(){
    return [0];
  }
}

// Dubins Car steering class that ever-naively turns towards the setpoint
class Dubins_Contr extends Controller {
  constructor(_robot,_Umax,_set){
    super(_robot);
    // Maximum steering allowed (scalar)
    this.Umax = _Umax;
    // Setpoint x,y-tuple (array)
    this.set = _set;

    // Create the marker that designates the robots current goalpoint
    this.goal = new PIXI.Text('X',{font : '24px Gill Sans', fill : this.robot.tint});
    this.goal.pivot.x = 10; this.goal.pivot.y = 12;
    this.goal.x = graphics.mapper.mapStateToPosition(this.set[0],this.set[1])[0];
    this.goal.y = graphics.mapper.mapStateToPosition(this.set[0],this.set[1])[1];
    stage.addChild(this.goal);
  }
  // Method for updating the setpoint to be tracked
  updateSetpoint(_set){
    this.set = _set;
    // Move the marker to match the new setpoint
    this.goal.x = graphics.mapper.mapStateToPosition(this.set[0],this.set[1])[0];
    this.goal.y = graphics.mapper.mapStateToPosition(this.set[0],this.set[1])[1];
  }
  // Returns the current control value responding to the robot's state
  u(){
    // Calculate the angle of the ray from the robot's current state to the
    // setpoint via the arctangent
    let deltaX = this.set[0] - this.robot.states[0];
    let deltaY = this.set[1] - this.robot.states[1];
    let trackAngle = Math.atan2(deltaY,deltaX);
    // Make the robot turn the shortest angular distance necessary to get to the
    // desired angle by wrapping to be between -pi and pi
    let angleDifference = this.robot.states[2] - trackAngle;
    angleDifference = Math.atan2(Math.sin(angleDifference),Math.cos(angleDifference));
    // Turn in the direction of the angular difference
    let Uout = 0
    if(angleDifference < 0){
      Uout = this.Umax;
    }
    if(angleDifference > 0){
      Uout = -this.Umax;
    }
    // Change setpoint if the robot has reached its goal

    // Return
    return [Uout];
  }
}

// Optimally safe controller class
class Safe_Contr extends Controller {
  constructor(_robot,_maxU){
    super(_robot);
    this.maxU = _maxU;
  }
  // Returns the current control value responding to the robot's state
  u(momentum){
    let u_out = [];
    //console.log(momentum);
    // For each control output...
    for(var curU=0;curU<this.robot.controlCoefficient().length;curU++){
      // maximize the Hamiltonian (f^T p) within the maximum output afforded
      if(this.dotProduct(this.robot.controlCoefficient()[curU], momentum)  > 0){
        u_out[curU] = this.maxU;
      }
      if(this.dotProduct(this.robot.controlCoefficient()[curU], momentum)  < 0){
        u_out[curU] = -this.maxU;
      }
    }
    return(u_out);
  }
  // Returns the inner product of two equal length arrays
  dotProduct(a,b){
    let sum = 0;
    for(var i=0;i<a.length;i++){
      sum += a[i] * b[i];
    }
    return(sum);
  }
}

// Intervention controller that swaps between PD and Safe controls
class Intervention_Contr extends Controller {
  constructor(_robot,_safeset,_maxU,_maxD,_tracker){
    super(_robot);
    // Default controller for when the system is not in danger
    this.tracker = _tracker;
    // Safety controller that will always steer the system out of danger
    this.safer   = new Safe_Contr(_robot,_maxU);
    // Safeset for determining danger status and deciding which control to use
    this.intervening_set = _safeset;
    // Added padding to the safeset for factoring in robot width
    this.trigger_level = 0;
  }
  // Method that returns the current input responding to the current state
  u(){
    // Check if the reachset value function is below the triggering level set
    if( this.intervening_set.value(this.robot.states) < this.trigger_level ){
      //console.log(this.intervening_set.value(this.robot.states),this.trigger_level);
      // If we have trespassed the reachset, interrupt with the safe policy
      return this.safer.u(this.intervening_set.gradV(this.robot.states) );
    }
    else{
      // If we're still safe, continue with the default tracking behavior
      return this.tracker.u();
    }
  }
}

// Intervention controller that swaps between PD and Safe controls
// interfaces with palettes of safesets
class PaletteIntervention_Contr extends Controller {
  constructor(_robot,_safesetPalette,_mySetID,_maxU,_maxD,_tracker,_record){
    super(_robot);
    // Default controller for when the system is not in danger
    this.tracker = _tracker;
    // Safety controller that will always steer the system out of danger
    this.safer   = new Safe_Contr(_robot,_maxU);
    // Safeset for determining danger status and deciding which control to use
    this.intervening_sets = _safesetPalette;
    // Which safeset to act on inside the palette
    this.setID = _mySetID;
    // Added padding to the safeset for factoring in robot width
    this.trigger_level = 0;
    // Record the initialization in goal and undetection
    if(_record){
      this.record = _record;
      record.goalSetEvents.push({
        robotID: this.robot.ID,
        undetection: this.intervening_sets.undetectionscape.slice(),
        goal: this.tracker.set,
        timestamp: clock
      });
    }
  }
  // Method that returns the current input responding to the current state
  u(){
    // Reset the tracker's goal if the goal has been reached
    let deltaX = this.tracker.set[0] - this.robot.states[0];
    let deltaY = this.tracker.set[1] - this.robot.states[1];
    if(Math.abs(deltaX) < 0.5 && Math.abs(deltaY) < 0.5){
      let swapX = this.tracker.set[0] * -1;
      let newGoal = graphics.mapper.randomStateXY();
      newGoal[0] = swapX;
      this.tracker.updateSetpoint(newGoal);
      if(ArcadeScore){
        ArcadeScore += 20;
        console.log(ArcadeScore);
      }
      this.intervening_sets.rerandomizeUndetection();
      // Record this change in goal and undetection
      if(this.record){
        record.goalSetEvents.push({
          robotID: this.robot.ID,
          undetection: this.intervening_sets.undetectionscape.slice(),
          goal: newGoal,
          timestamp: clock
        });
      }
    }
    // Check if the reachset value function is below the triggering level set
    if( this.intervening_sets.value(this.setID,this.robot.states) < this.trigger_level ){
      //console.log(this.intervening_set.value(this.robot.states),this.trigger_level);
      // If we have trespassed the reachset, interrupt with the safe policy
      return this.safer.u(this.intervening_sets.gradV(this.setID,this.robot.states) );
    }
    else{
      // If we're still safe, continue with the default tracking behavior
      return this.tracker.u();
    }
  }
}

// Intervention controller that swaps between PD and Safe controls
// interfaces with palettes of safesets
class PaletteLegibilization_Contr extends Controller {
  constructor(_robot,_safesetPalette,_mySetID,_maxU,_maxD,_tracker,_record){
    super(_robot);
    // Default controller for when the system is not in danger
    this.tracker = _tracker;
    // Safety controller that will always steer the system out of danger
    this.safer   = new Safe_Contr(_robot,_maxU);
    // Safeset for determining danger status and deciding which control to use
    this.intervening_sets = _safesetPalette;
    // Which safeset to act on inside the palette
    this.setID = _mySetID;
    // Added padding to the safeset for factoring in robot width
    this.trigger_level = 0;
    // Record the initialization in goal and undetection
    if(_record){
      this.record = _record;
      record.goalSetEvents.push({
        robotID: this.robot.ID,
        undetection: this.intervening_sets.undetectionscape.slice(),
        goal: this.tracker.set,
        timestamp: clock
      });
    }
  }
  // Method that returns the current input responding to the current state
  u(){
    // Reset the tracker's goal if the goal has been reached
    let deltaX = this.tracker.set[0] - this.robot.states[0];
    let deltaY = this.tracker.set[1] - this.robot.states[1];
    if(Math.abs(deltaX) < 0.5 && Math.abs(deltaY) < 0.5){
      let swapX = this.tracker.set[0] * -1;
      let newGoal = graphics.mapper.randomStateXY();
      newGoal[0] = swapX;
      this.tracker.updateSetpoint(newGoal);
      if(ArcadeScore){
        ArcadeScore += 20;
        console.log(ArcadeScore);
      }
      this.intervening_sets.rerandomizeUndetection();
      // Record this change in goal and undetection
      if(this.record){
        record.goalSetEvents.push({
          robotID: this.robot.ID,
          undetection: this.intervening_sets.undetectionscape.slice(),
          goal: newGoal,
          timestamp: clock
        });
      }
    }
    // Return the legible control
    console.log(this.setID)
    let gradJ_robotSafe = this.intervening_sets.gradV(0,this.robot.states);
    console.log(gradJ_robotSafe);
    let gradJ_humanSafe = this.intervening_sets.gradV(4,this.robot.states);
    console.log(gradJ_humanSafe);
    let gradLegible = []
    let mu1 = 10; let mu2 = 1;
    for(var i = 0; i < gradJ_humanSafe.length; i++) {
        gradLegible[i] = mu1*gradJ_humanSafe[i] - mu2*gradJ_robotSafe[i];
    }
    return this.safer.u(gradLegible);
  }
}
