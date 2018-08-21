
var nav2 = {};

// Constants
nav2.maxAccel = 2.0;       // meters per second squared
nav2.maxSpeed = 1.0;       // meters per second
nav2.maxRotAccel = 0.7;    // radians per second squared
nav2.errorHalfLife = 0.3;  // seconds
nav2.k = Math.LN2 / nav2.errorHalfLife;
nav2.rotErrorHalfLife = 0.15;  // seconds
nav2.rotK = Math.LN2 / nav2.rotErrorHalfLife;
nav2.maxAccelTime = nav2.maxSpeed / nav2.maxAccel;
nav2.accelDist = nav2.maxSpeed * 0.5 * nav2.maxAccelTime;
nav2.tiltSpeed = 0.5;      // radians per second
nav2.minTilt = -1.2;       // radians
nav2.maxTilt = 1.2;        // radians

nav2.reset=(function() {
    this.pos = [ 0.0, 0.0 ];       // meters
    this.orientation = 0.0;        // radians
    this.vel = [ 0.0, 0.0 ];       // meters per second
    this.tilt = 0.0;               // radians
    this.rotSpeed = 0.0;           // radians per second
    this.targetOrientation = 0.0;  // radians
    this.targetVel = [ 0.0, 0.0 ]; // meters per second
    this.targetTilt = 0.0;         // radians
    this.lastUpdate = new Date().getTime() / 1000;   // seconds
    this.mode = 0;
    this.rotMode = 0;
    this.path = null;
    this.pathLen = 0;
    this.pathStartTime = 0;
    this.initialVel = [ 0.0, 0.0 ];
    this.initialPos = [ 0.0, 0.0 ];
});

nav2.update=(function() {
    var now = new Date().getTime();
    var dt = now / 1000 - nav2.lastUpdate;

    if(nav2.mode) {
        // Time elapsed since path start
        var t = (now - nav2.pathStartTime) / 1000;

        var timeAtMaxSpeed = Math.max(0,
            (nav2.pathLen - 2.0 * nav2.accelDist) / nav2.maxSpeed);
        var accelTime = (timeAtMaxSpeed > 0 ? nav2.maxAccelTime : 
            Math.sqrt(nav2.pathLen / nav2.maxAccel));
        var accelDist = accelTime * accelTime * nav2.maxAccel * 0.5;
        var totalTime = accelTime + timeAtMaxSpeed + accelTime;

        // How far would we have travelled if we followed the path perfectly?
        var speed,d;
        if( t < accelTime) {
            speed = t * nav2.maxAccel;
            d = t*speed*0.5;
        } else if( t < accelTime + timeAtMaxSpeed) {
            speed = nav2.maxSpeed;
            d = accelDist + (t-accelTime) * nav2.maxSpeed;
        } else {
            var remainingTime = Math.max(totalTime - t,0);
            speed = remainingTime * nav2.maxAccel;
            var remainingDist = remainingTime * speed * 0.5;
            d = nav2.pathLen - remainingDist;
        }
 
        // What would the position have been if we followed the path perfectly?
        var segment = 1;
        var perfectPos = [0,0];
        while( segment < nav2.path.length) {
            var dx = nav2.path[segment][0] - nav2.path[segment-1][0];
            var dy = nav2.path[segment][1] - nav2.path[segment-1][1];
            var segLen = Math.sqrt(dx*dx+dy*dy);
            perfectPos[0] = nav2.path[segment-1][0] + dx * d / segLen;
            perfectPos[1] = nav2.path[segment-1][1] + dy * d / segLen;

            // Even though we're not using it, set the velocity
            // to something reasonable, in case we switch modes.
            nav2.vel[0] = speed * dx / segLen;
            nav2.vel[1] = speed * dy / segLen;

            if( segLen >= d) break;
            d -= segLen;
            segment++;
        }
 
        // What would the position have been if we had maintained
        // our initial velocity?
        var projectedPos = [
            nav2.initialPos[0] + t * nav2.initialVel[0],
            nav2.initialPos[1] + t * nav2.initialVel[1]];

        // Blend the perfect position with the projected position,
        // with an exponential error decay rate.
        var posError = [
            projectedPos[0] - perfectPos[0],
            projectedPos[1] - perfectPos[1]
        ];

        var adjust = Math.min(t,1);
        adjust *= adjust * (3 - 2*adjust);
        nav2.pos[0] = projectedPos[0] - posError[0] * adjust;
        nav2.pos[1] = projectedPos[1] - posError[1] * adjust;
    } else {
        // Compute the speed error
        var velError = [
            nav2.targetVel[0] - nav2.vel[0],
            nav2.targetVel[1] - nav2.vel[1]
        ];

        // Compute the new speed
        var adjust = Math.exp(-nav2.k*dt);
        nav2.vel[0] = nav2.targetVel[0] - velError[0] * adjust;
        nav2.vel[1] = nav2.targetVel[1] - velError[1] * adjust;

        // Compute the new position
        var s = (1.0 - adjust) / nav2.k;
        nav2.pos[0] += nav2.targetVel[0] * dt - velError[0] * s;
        nav2.pos[1] += nav2.targetVel[1] * dt - velError[1] * s;
    }

    if(nav2.rotMode) {
        // Compute the rotational speed error
        var rotSpeedError = nav2.targetRotSpeed - nav2.rotSpeed;

        // Compute the new rotational speed
        var adjust = Math.exp(-nav2.rotK*dt);
        nav2.rotSpeed = nav2.targetRotSpeed - rotSpeedError * adjust;

        // Compute the new orientation
        var s = (1.0 - adjust) / nav2.rotK;
        nav2.orientation += nav2.targetRotSpeed * dt - rotSpeedError * s;
        nav2.targetOrientation = nav2.orientation;
    } else {
        var prevOError = nav2.targetOrientation - nav2.orientation;

        // Adjust the orientation
        if( prevOError != 0) nav2.orientation += nav2.rotSpeed * dt;

        var oError = nav2.targetOrientation - nav2.orientation;

        if( prevOError * oError < 0.0) {
            nav2.orientation = nav2.targetOrientation;
            oError = 0.0;
        }

        if( oError > 0) {
            nav2.rotSpeed += nav2.maxRotAccel * dt;

            // Make sure we won't overshoot the target orientation
            if( oError < nav2.rotSpeed * nav2.rotSpeed / (2.0 * nav2.maxRotAccel)) {
                nav2.rotSpeed = Math.sqrt(2.0 * nav2.maxRotAccel * oError);
            }
        } else if( oError < 0) {
            nav2.rotSpeed -= nav2.maxRotAccel * dt;

            // Make sure we won't overshoot the target orientation
            if( -oError < nav2.rotSpeed * nav2.rotSpeed / (2.0 * nav2.maxRotAccel)) {
                nav2.rotSpeed = -Math.sqrt(2.0 * nav2.maxRotAccel * -oError);
            }
        }
    }

    if( nav2.tilt < nav2.targetTilt) {
        nav2.tilt += nav2.tiltSpeed * dt;
        if( nav2.tilt > nav2.targetTilt) nav2.tilt = nav2.targetTilt;
        if( nav2.tilt > nav2.maxTilt) nav2.tilt = nav2.maxTilt;
    } else {
        nav2.tilt -= nav2.tiltSpeed * dt;
        if( nav2.tilt < nav2.targetTilt) nav2.tilt = nav2.targetTilt;
        if( nav2.tilt < nav2.minTilt) nav2.tilt = nav2.minTilt;
    }

    nav2.lastUpdate = now / 1000;
});

nav2.stop=(function() {
    nav2.update();
    nav2.vel = [ 0.0, 0.0 ];
    nav2.targetVel = [ 0.0, 0.0 ];
    nav2.rotSpeed = 0.0;
    nav2.targetOrientation = nav2.orientation;
    nav2.mode = 0;
});

nav2.setTargetOrientation=(function(orientation)
{
    nav2.update();
    nav2.targetOrientation = orientation;
    nav2.targetRotSpeed = 0;
    nav2.rotMode = 0;
});

nav2.setTargetRotSpeed=(function(rotSpeed)
{
    nav2.update();
    nav2.targetRotSpeed = rotSpeed;
    nav2.targetOrientation = nav2.orientation;
    nav2.rotMode = 1;
});

nav2.setTargetVelocity=(function(velocity)
{
    nav2.update();
    nav2.targetVel = velocity;
    nav2.mode = 0;
});

nav2.setPosition=(function(pos,orientation) {
    nav2.update();
    nav2.pos[0] = pos[0];
    nav2.pos[1] = pos[1];
    nav2.orientation = orientation;
});

nav2.setTargetHeadTilt=(function(tilt) {
    nav2.update();
    nav2.targetTilt = tilt;
});

nav2.setTargetPath=(function(path) {
    nav2.update();

    nav2.path = new Array();
    nav2.pathLen = 0;
    nav2.mode = 1;
    nav2.pathStartTime = new Date().getTime();
    nav2.initialVel[0] = nav2.vel[0];
    nav2.initialVel[1] = nav2.vel[1];
    nav2.initialPos[0] = nav2.pos[0];
    nav2.initialPos[1] = nav2.pos[1];
    for( var i=0; i < path.length; ++i) {
        nav2.path.push(path[i]);
        if( i > 0) {
            var dx = path[i][0]-path[i-1][0];
            var dy = path[i][1]-path[i-1][1];
            nav2.pathLen += Math.sqrt(dx*dx+dy*dy);
        }
    }
});

nav2.reset();
