/* global Ammo */
import CONSTANTS from "../constants.js";
const CONSTRAINT = CONSTANTS.CONSTRAINT;
const CONSTRAINTS = [
  CONSTRAINT.LOCK,
  CONSTRAINT.FIXED,
  CONSTRAINT.SPRING,
  CONSTRAINT.SLIDER,
  CONSTRAINT.HINGE,
  CONSTRAINT.CONE_TWIST,
  CONSTRAINT.POINT_TO_POINT
];

/**
 * @return {Ammo.btTypedConstraint}
 */
const Constraint = function(constraintConfig, body, targetBody, world) {
  this.physicsConstraint;

  this.world = world;

  const type =
    constraintConfig.type && CONSTRAINTS.indexOf(constraintConfig.type) ? constraintConfig.type : CONSTRAINT.LOCK;

  const bodyTransform = body.physicsBody
    .getCenterOfMassTransform()
    .inverse()
    .op_mul(targetBody.physicsBody.getWorldTransform());
  const targetTransform = new Ammo.btTransform();
  targetTransform.setIdentity();

  //let bodyOrigin = bodyTransform.getOrigin();
  //console.log("body transform origin: " + JSON.stringify(bodyOrigin));
  if (constraintConfig.pivot != undefined) {
    let pivot = new Ammo.btVector3(constraintConfig.pivot.x,constraintConfig.pivot.y,constraintConfig.pivot.z);
    bodyTransform.setOrigin(pivot);
    Ammo.destroy(pivot);
  } 
  if (constraintConfig.targetPivot != undefined) {
    let targetPivot = new Ammo.btVector3(constraintConfig.targetPivot.x,constraintConfig.targetPivot.y,constraintConfig.targetPivot.z);
    targetTransform.setOrigin(targetPivot);
    Ammo.destroy(targetPivot);
  }
  //console.log("body transform origin: " + JSON.stringify(targetTransform.getOrigin()));

  switch (type) {
    case CONSTRAINT.LOCK: {
      this.physicsConstraint = new Ammo.btGeneric6DofConstraint(
        body.physicsBody,
        targetBody.physicsBody,
        bodyTransform,
        targetTransform,
        true
      );
      const zero = new Ammo.btVector3(0, 0, 0);
      //TODO: allow these to be configurable
      if (constraintConfig.linearLow != undefined) {
        let vec = new Ammo.btVector3(constraintConfig.linearLow.x, constraintConfig.linearLow.y, constraintConfig.linearLow.z);
        this.physicsConstraint.setLinearLowerLimit(vec);
        Ammo.destroy(vec);
      } else this.physicsConstraint.setLinearLowerLimit(zero);
      if (constraintConfig.linearHigh != undefined) {
        let vec = new Ammo.btVector3(constraintConfig.linearHigh.x, constraintConfig.linearHigh.y, constraintConfig.linearHigh.z);
        this.physicsConstraint.setLinearUpperLimit(vec);
        Ammo.destroy(vec);
      } else this.physicsConstraint.setLinearUpperLimit(zero);
      if (constraintConfig.angularLow != undefined) {
        let vec = new Ammo.btVector3(constraintConfig.angularLow.x, constraintConfig.angularLow.y, constraintConfig.angularLow.z);
        this.physicsConstraint.setAngularLowerLimit(vec);
        Ammo.destroy(vec);
      } else this.physicsConstraint.setAngularLowerLimit(zero);
      if (constraintConfig.angularHigh != undefined) {
        let vec = new Ammo.btVector3(constraintConfig.angularHigh.x, constraintConfig.angularHigh.y, constraintConfig.angularHigh.z);
        this.physicsConstraint.setAngularUpperLimit(vec);
        Ammo.destroy(vec);
      } else this.physicsConstraint.setAngularUpperLimit(zero);
      
      //NOPE!!! This may have been left out of ammo.js from bullet. Or maybe the version of bullet we got ammo js from. Or something. But it is not here.
      //let angXMotor = this.physicsConstraint.getRotationalLimitMotor(0);//maybe?
      //if (angXMotor != undefined) {
      //  console.log("Got a rotational motor?");
        //angXMotor.m_targetVelocity = 2.0;//maybe???
      //}

      Ammo.destroy(zero);
      break;
    }

    //TODO: test and verify all other constraint types
    case CONSTRAINT.FIXED: {
      //btFixedConstraint does not seem to debug render
      bodyTransform.setRotation(body.physicsBody.getWorldTransform().getRotation());
      targetTransform.setRotation(targetBody.physicsBody.getWorldTransform().getRotation());
      this.physicsConstraint = new Ammo.btFixedConstraint(
        body.physicsBody,
        targetBody.physicsBody,
        bodyTransform,
        targetTransform
      );
      break;
    }
    case CONSTRAINT.SPRING: {
      this.physicsConstraint = new Ammo.btGeneric6DofSpringConstraint(
        body.physicsBody,
        targetBody.physicsBody,
        bodyTransform,
        targetTransform,
        true
      );
      //TODO: enableSpring, setStiffness and setDamping
      break;
    }
    case CONSTRAINT.SLIDER: {
      //TODO: support setting linear and angular limits
      this.physicsConstraint = new Ammo.btSliderConstraint(
        body.physicsBody,
        targetBody.physicsBody,
        bodyTransform,
        targetTransform,
        true
      );
      this.physicsConstraint.setLowerLinLimit(-1);
      this.physicsConstraint.setUpperLinLimit(1);
      // this.physicsConstraint.setLowerAngLimit();
      // this.physicsConstraint.setUpperAngLimit();
      break;
    }
    case CONSTRAINT.HINGE: {
      if (!constraintConfig.pivot) {
        throw new Error("pivot must be defined for type: hinge");
      }
      if (!constraintConfig.targetPivot) {
        throw new Error("targetPivot must be defined for type: hinge");
      }
      if (!constraintConfig.axis) {
        throw new Error("axis must be defined for type: hinge");
      }
      if (!constraintConfig.targetAxis) {
        throw new Error("targetAxis must be defined for type: hinge");
      }

      const pivot = new Ammo.btVector3(constraintConfig.pivot.x, constraintConfig.pivot.y, constraintConfig.pivot.z);
      const targetPivot = new Ammo.btVector3(
        constraintConfig.targetPivot.x,
        constraintConfig.targetPivot.y,
        constraintConfig.targetPivot.z
      );

      const axis = new Ammo.btVector3(constraintConfig.axis.x, constraintConfig.axis.y, constraintConfig.axis.z);
      const targetAxis = new Ammo.btVector3(
        constraintConfig.targetAxis.x,
        constraintConfig.targetAxis.y,
        constraintConfig.targetAxis.z
      );

      this.physicsConstraint = new Ammo.btHingeConstraint(
        body.physicsBody,
        targetBody.physicsBody,
        pivot,
        targetPivot,
        axis,
        targetAxis,
        false
      );//last one true or false? useReferenceFrameA, defaults to false.

      //NEW, from MeTabi:
      console.log("Joint limits:  high " + constraintConfig.limitHigh + " low " + constraintConfig.limitLow + " " +
                  "  velocity " + constraintConfig.motorVelocity + " impulse " + constraintConfig.motorImpulse);
      let limitHigh, limitLow;
      if (constraintConfig.limitHigh != undefined) {
        limitHigh = constraintConfig.limitHigh;
        if (constraintConfig.limitLow != undefined) {
          limitLow = constraintConfig.limitLow;
        } else {
          limitLow = constraintConfig.limitHigh * -1;
        }
        this.physicsConstraint.setLimit(limitLow, limitHigh, 0.9, 0.3, 1);
        //this.physicsConstraint.setLimit(-Math.PI/4, Math.PI/4, 0.9, 0.3, 1);
      }
      if (constraintConfig.motorVelocity != undefined) {
        let velocity = constraintConfig.motorVelocity;
        let maxImpulse = 5;
        if (constraintConfig.motorImpulse != undefined) {
          maxImpulse = constraintConfig.motorImpulse;
        }
        this.physicsConstraint.enableAngularMotor(true, velocity, maxImpulse);
      }
      ///////////////////////

      Ammo.destroy(pivot);
      Ammo.destroy(targetPivot);
      Ammo.destroy(axis);
      Ammo.destroy(targetAxis);
      break;
    }
    case CONSTRAINT.CONE_TWIST: {
      if (!constraintConfig.pivot) {
        throw new Error("pivot must be defined for type: cone-twist");
      }
      if (!constraintConfig.targetPivot) {
        throw new Error("targetPivot must be defined for type: cone-twist");
      }

      //const pivotTransform = new Ammo.btTransform();
      //pivotTransform.setIdentity();
      //pivotTransform
      //  .getOrigin()
      //  .setValue(constraintConfig.targetPivot.x, constraintConfig.targetPivot.y, constraintConfig.targetPivot.z);
      //this.physicsConstraint = new Ammo.btConeTwistConstraint(body.physicsBody, pivotTransform);//Wha?
      this.physicsConstraint = new Ammo.btConeTwistConstraint(
        body.physicsBody,
        targetBody.physicsBody,
        bodyTransform,
        targetTransform
      );
      
      //setLimit: this is set up so that the first axis (3) is Twist, and the other two are Swing1 and Swing2. Not sure which is which.
      if (constraintConfig.angularHigh != undefined) {
        if (constraintConfig.angularHigh.y > 0) {
          this.physicsConstraint.setLimit(3,constraintConfig.angularHigh.y);
        }
        if (constraintConfig.angularHigh.x > 0) { 
          this.physicsConstraint.setLimit(4,constraintConfig.angularHigh.x);
        }
        if  (constraintConfig.angularHigh.z > 0) {
          this.physicsConstraint.setLimit(5,constraintConfig.angularHigh.z);
        }
      }
      //this.physicsConstraint.enableMotor(true);
      //let q = new Ammo.btQuaternion(0,0,0.707,0.707);
      //this.physicsConstraint.setMaxMotorImpulse(1.0);
      //this.physicsConstraint.setMotorTarget(q);//maybe???
      //Ammo.destroy(q);
      break;
    }


    case CONSTRAINT.POINT_TO_POINT: {
      if (!constraintConfig.pivot) {
        throw new Error("pivot must be defined for type: point-to-point");
      }
      if (!constraintConfig.targetPivot) {
        throw new Error("targetPivot must be defined for type: point-to-point");
      }

      const pivot = new Ammo.btVector3(constraintConfig.pivot.x, constraintConfig.pivot.y, constraintConfig.pivot.z);
      const targetPivot = new Ammo.btVector3(
        constraintConfig.targetPivot.x,
        constraintConfig.targetPivot.y,
        constraintConfig.targetPivot.z
      );

      this.physicsConstraint = new Ammo.btPoint2PointConstraint(
        body.physicsBody,
        targetBody.physicsBody,
        pivot,
        targetPivot
      );

      Ammo.destroy(pivot);
      Ammo.destroy(targetPivot);
      break;
    }
  }

  Ammo.destroy(targetTransform);

  this.world.physicsWorld.addConstraint(this.physicsConstraint, false);
};

Constraint.prototype.destroy = function() {
  if (!this.physicsConstraint) return;

  this.world.physicsWorld.removeConstraint(this.physicsConstraint);
  Ammo.destroy(this.physicsConstraint);
  this.physicsConstraint = null;
};

Constraint.prototype.update = function(options) {
  if (!this.physicsConstraint) return;
  console.log("physics constraint updating, type  =  " + options.type);
  if (options.motorVelocity != undefined) { //(this.physicsConstraint.type == "hinge") {

    if (options.motorVelocity != undefined) { //Hm, are these only for hinge joints, though?
      //this.physicsConstraint.setMotorTargetVelocity(options.motorVelocity);
      let motorImpulse = 3; //this.physicsConstraint.getMaxMotorImpulse();//NOPE!! 
      if (options.motorImpulse != undefined) {
        motorImpulse = options.motorImpulse;
      }
      this.physicsConstraint.enableAngularMotor(true,options.motorVelocity,motorImpulse);
    }
    if (options.motorImpulse != undefined && options.motorVelocity == undefined) {
      let motorVelocity = 1;//this.physicsConstraint.getMotorTargetVelocity();//NOPE!! 
      //this.physicsConstraint.setMaxMotorImpulse(options.motorImpulse);
      this.physicsConstraint.enableAngularMotor(true,motorVelocity,options.motorImpulse);
    }
    if (options.limitHigh != undefined) {
      let limitHigh = options.limitHigh;
      let limitLow = limitHigh * -1;
      if (options.limitLow != undefined) {
        limitLow = options.limitLow;
      }
      this.physicsConstraint.setLimit(limitLow,limitHigh,0.9,0.3,1.0);
    }
  } else if (options.motorTarget != undefined) { //(this.physicsConstraint.type == "coneTwist") {
    if (options.motorImpulse != undefined || options.motorTarget != undefined) {
      let motorImpulse = 1;
      if (options.motorImpulse != undefined) {
        motorImpulse = options.motorImpulse;
      }
      this.physicsConstraint.enableMotor(true);
      this.physicsConstraint.setMaxMotorImpulse(motorImpulse);
      if (options.motorTarget != undefined) { //copy ammo quaternion from three quaternion
        let q = new Ammo.btQuaternion(options.motorTarget[0],options.motorTarget[1],options.motorTarget[2],options.motorTarget[3]);
        this.physicsConstraint.setMotorTarget(q);
        Ammo.destroy(q);
      } else {
        let q = new Ammo.btQuaternion(0,0,0,1);
        this.physicsConstraint.setMotorTarget(q);
        Ammo.destroy(q);
      }
    }
  }
};

export default Constraint;
