// Virtual class defining the interface for obstacle objects
class Obstacle {
  constructor(_ObX,_ObY,avoidSets){
    // save a reference to the reachable set conforming to this shaped obstacle
    // the reachable set's state space is relative to the obstacle's position
    this.avoidSets = avoidSets;
    // Store the offset position for this obstacle
    this.ObX = _ObX;
    this.ObY = _ObY;
    this.offset = [this.ObX,this.ObY];
    this.collisionSet = new SafeSet;
  }
  // Method for displaying the obstacle
  render(){
    return;
  }
  // Method for displaying the obstacle with added width for level-set picking
  renderAugmented(pad){
    return;
  }
  // Method for transforming global state to obstacle-relative state
  offsetStates(states){
    let offsetS = states.slice();
    for(let curDim = 0; curDim < states.length; curDim++){
      offsetS[curDim] -= this.offset[curDim];
    }
    return offsetS;
  }
  // Access method for determining collision in global coordinates
  collisionSetValue(states){
    return this.collisionSet.value(this.offsetStates(states));
  }
  // Passing calls to internalized reachable sets palette
  value(setID,states){
    return this.avoidSets.value(setID,this.offsetStates(states));
  }
  // Method for calculating the gradient
  gradV(setID,states){
    return this.avoidSets.gradV(setID,this.offsetStates(states));
  }
  // Method for displaying the value function on a grid
  displayGrid(setID,graphics,color,currentState,sweptStateX,sweptStateY){
    this.avoidSets.displayGrid(setID,graphics,color,
        this.offsetStates(currentState),sweptStateX,sweptStateY,this.offset);
    return 0;
  }
}

// Defines the landscape of obstacles that the robots must dodge
// This is effectively a wrapper class for an array of Obstacle objects
class Obstaclescape {
  constructor(_obstacles){
    // create the lists of obstacles and their access modifiers
    this.obstacles = _obstacles;
    this.obstacleDestroyed = [];
    this.obstacleUndetected = [];
    // Initialize the access modifiers to default accessible
    for(let obNum = 0; obNum < this.obstacles.length ; obNum++){
      this.obstacleDestroyed[obNum] = false;
      this.obstacleUndetected[obNum] = false;
    }
  }
  // Method for displaying the obstacle
  render(){
    for(let obNum = 0; obNum < this.obstacles.length ; obNum++){
      if(this.obstacleDestroyed[obNum] == false){
        this.obstacles[obNum].render();
      }
    }
    return;
  }
  // Method for displaying the obstacle with added width for level-set picking
  renderAugmented(pad){
    for(let obNum = 0; obNum < this.obstacles.length ; obNum++){
      if(this.obstacleDestroyed[obNum] == false){
        this.obstacles[obNum].renderAugmented(pad);
      }
    }
    return;
  }
  // Passing calls to all listed obstacles' value functions and choosing the
  // worst safety value (union of safety sets)
  value(setID,states){
    // Running minimum value (initialized to preposterouly large number... 100)
    let curMinValue = 100;
    // Iterate over obstacles and find the lowest value function
    for(let obNum = 0; obNum < this.obstacles.length ; obNum++){
      if(this.obstacleDestroyed[obNum] == false
            && this.obstacleUndetected[obNum] == false){
        let obsValue = this.obstacles[obNum].value(setID,states);
        if(obsValue < curMinValue)
          curMinValue = obsValue;
      }
    }
    // Return the lowest reachset value amongst all obstacles
    return curMinValue;
  }
  // Determine collision in global coordinates with any of the contained obstacles
  collisionSetValue(states){
    // Running minimum value (initialized to preposterouly large number... 100)
    let curMinValue = 100;
    let curMinOb = 0;
    // Iterate over obstacles and find the lowest value function
    for(let obNum = 0; obNum < this.obstacles.length ; obNum++){
      if(this.obstacleDestroyed[obNum] == false
          && this.obstacleUndetected[obNum] == true
          ){
        let obsValue = this.obstacles[obNum].collisionSetValue(states);
        if(obsValue < curMinValue){
          curMinValue = obsValue;
          curMinOb = obNum;
        }
      }
    }
    // Return the lowest reachset value amongst all obstacles
    return [curMinValue,curMinOb];
  }
  // Return the gradient corresponding to whichever obstacle currently dominates
  // the union by having the worst safety-value function
  gradV(setID,states){
    // Running minimum value (initialized to preposterouly large number... 100)
    let curMinValue = 100;
    let dominantObstacle = 0;
    // Iterate over obstacles to find the obstacle that has the lowest value
    // function at this current state
    for(let obNum = 0; obNum < this.obstacles.length ; obNum++){
      if(this.obstacleDestroyed[obNum] == false
            && this.obstacleUndetected[obNum] == false){
        let obsValue = this.obstacles[obNum].value(setID,states);
        if(obsValue < curMinValue){
          curMinValue = obsValue;
          dominantObstacle = obNum;
        }
      }
    }
    // Return the gradient corresponding to that value function
    return this.obstacles[dominantObstacle].gradV(setID,states);
  }
  // Display the grid for every listed obstacle that hasn't been destroyed and
  // doesn't have a suppressed value function ((WARNING: slows down the app))
  displayGrid(setID,graphics,color,currentState,sweptStateX,sweptStateY){
    for(let obNum = 0; obNum < this.obstacles.length ; obNum++){
      if(this.obstacleDestroyed[obNum] == false
            && this.obstacleUndetected[obNum] == false){
        this.obstacles[obNum].displayGrid(setID,graphics,color,currentState
                                                      ,sweptStateX,sweptStateY);
      }
    }
    return 0;
  }
}

class maskedObstaclescape {
  constructor(_obstaclescape){
    // save a reference to the reachable set conforming to this shaped obstacle
    // the reachable set's state space is relative to the obstacle's position
    this.obstaclescape = _obstaclescape;
    this.undetectionscape = [];
    this.rerandomizeUndetection();
  }
  // Method for displaying the obstacle
  render(){
    this.obstaclescape.render();
    return;
  }
  // Method for displaying the obstacle with added width for level-set picking
  renderAugmented(pad){
    this.obstaclescape.renderAugmented(pad);
    return;
  }
  //
  rerandomizeUndetection(){
    for(let obNum = 0; obNum < this.obstaclescape.obstacles.length ; obNum++){
      let undetected = true;
      if(random() < 0.8){
        undetected = false;
      }
      this.undetectionscape[obNum] = undetected;
    }
    return ;
  }
  // Passing calls to internalized collision set values
  collisionSetValue(states){
    this.obstaclescape.obstacleUndetected = this.undetectionscape.slice();
    return this.obstaclescape.collisionSetValue(states);
  }
  // Passing calls to internalized reachable sets palette
  value(setID,states){
    this.obstaclescape.obstacleUndetected = this.undetectionscape.slice();
    return this.obstaclescape.value(setID,states);
  }
  // Access method for determining collision in global coordinates
  collisionSetValue(states){
    this.obstaclescape.obstacleUndetected = this.undetectionscape.slice();
    return this.obstaclescape.collisionSetValue(states);
  }
  // Method for calculating the gradient
  gradV(setID,states){
    this.obstaclescape.obstacleUndetected = this.undetectionscape.slice();
    return this.obstaclescape.gradV(setID,states);
  }
  // Method for displaying the value function on a grid
  displayGrid(setID,graphics,color,currentState,sweptStateX,sweptStateY){
    this.obstaclescape.displayGrid(setID,graphics,color,
        currentState,sweptStateX,sweptStateY);
    return 0;
  }
}

// Rectangular obstacle designed for decoupled double integrator system
class BoxObstacle extends Obstacle{
  constructor(_ObX,_ObY,_ObW,_ObH,avoidSets){
    super(_ObX,_ObY,avoidSets);
    this.ObW = _ObW;
    this.ObH = _ObH;
    // Offset vector (in the state space) that translates between global
    // coordinates and obstacle-relative coordinates for safeset
    this.offset = [this.ObX,0,this.ObY,0];
    // Define a collision set for determining crashes and clicks
    this.collisionSet =
        new twoTwo(new dubIntInterval_Set(_ObW),new dubIntInterval_Set(_ObH) );
  }
  // Rendering standard obstacle
  render(){
    this.drawFromState(graphics,5,0x4C1C13,
        this.ObX-this.ObW,this.ObY-this.ObH,this.ObX+this.ObW,this.ObY+this.ObH);
    return;
  }
  // Rendering an obstacle with extra buffer distance added to all sides
  renderAugmented(pad){
    this.drawFromState(graphics,0,0xcf4c34,
        this.ObX-this.ObW-pad,this.ObY-this.ObH-pad,
        this.ObX+this.ObW+pad,this.ObY+this.ObH+pad);
    this.render();
    return;
  }
  // Draw a quadrilateral given the state coordinates of the edges
  drawFromState(graphics,linewidth,color, left,top,right,bottom){
    // Scale from state coordinates to screen coordinates
    let topleft = graphics.mapper.mapStateToPosition(left,top);
    let bottomright = graphics.mapper.mapStateToPosition(right,bottom);
    // Draw teh filled rectangle
    this.drawQuadrilateral(graphics,linewidth,color,
        topleft[0],topleft[1],bottomright[0],bottomright[1]);
    return;
  }
  // Draw a quadrilateral using PIXI.graphics
  drawQuadrilateral(graphics,linewidth,color,left,top,right,bottom){
    // Set a fill and line style
    graphics.beginFill(color);
    graphics.lineStyle(linewidth, 0x000000);

    // Draw the quadrilateral
    graphics.moveTo(left,top);
    graphics.lineTo(right,top);
    graphics.lineTo(right,bottom);
    graphics.lineTo(left,bottom);
    graphics.endFill();
    return;
  }
}

// Circular obstacle designed for Dubins-Car system
class RoundObstacle extends Obstacle{
  constructor(_ObX,_ObY,_ObR,radiusTrim,avoidSets){
    super(_ObX,_ObY,avoidSets);
    this.ObR = _ObR;
    this.trimmedObR = _ObR - radiusTrim;
    this.color = 0x000000;
    // Offset vector (in the state space) that translates between global
    // coordinates and obstacle-relative coordinates for safeset
    this.offset = [_ObX,_ObY,0]; // TODO: Make obstacles system agnostic
    // Define a (loose) collision set for determining crashes and clicks
    this.collisionSet = new dubinsCircle_Set(_ObR*0.95);
  }
  // Rendering standard obstacle
  render(){
    // Color used to be 0x4C1C13
    this.drawFromState(graphics,5,this.color, this.ObX,this.ObY,this.trimmedObR);
    return;
  }
  // Rendering an obstacle with extra buffer distance added to all sides
  renderAugmented(pad){
    this.drawFromState(graphics,0,0xcf4c34, this.ObX,this.ObY,this.trimmedObR+pad);
    this.render();
    return;
  }
  // Draw a circle given the state coordinates of its center and radius
  drawFromState(graphics,linewidth,color, _x,_y,radius){
    let center = graphics.mapper.mapStateToPosition(_x,_y);
    this.drawCircle(graphics,linewidth,color,
        center[0],center[1],radius*graphics.mapper.Mxx);
    return;
  }
  // Draw a circle using PIXI.graphics
  drawCircle(graphics,linewidth,color,left,top,radius){
    // Set a fill and line style
    graphics.beginFill(color);
    graphics.lineStyle(linewidth, 0x000000);

    // Draw the circle
    graphics.drawCircle(left,top,radius);
    graphics.endFill();
    return;
  }
}
