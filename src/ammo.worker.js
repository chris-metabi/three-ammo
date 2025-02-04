import CONSTANTS from "../constants.js";
const MESSAGE_TYPES = CONSTANTS.MESSAGE_TYPES;
const TYPE = CONSTANTS.TYPE;
const SHAPE = CONSTANTS.SHAPE;
const CONSTRAINT = CONSTANTS.CONSTRAINT;
const BUFFER_CONFIG = CONSTANTS.BUFFER_CONFIG;
const BUFFER_STATE = CONSTANTS.BUFFER_STATE;
import * as THREE from "three";
import World from "./world";
import Body from "./body";
import Constraint from "./constraint";
import { DefaultBufferSize } from "ammo-debug-drawer";

import { createCollisionShapes } from "three-to-ammo";

import Ammo from "ammo.js/builds/ammo.wasm.js";
import AmmoWasm from "ammo.js/builds/ammo.wasm.wasm";

function initializeWasm(wasmUrl) {
  return Ammo.bind(undefined, {
    locateFile(path) {
      if (path.endsWith(".wasm")) {
        if (wasmUrl) {
          return wasmUrl;
        } else {
          return new URL(AmmoWasm, location.origin).href;
        }
      }
      return path;
    }
  });
}

const uuids = [];
const bodies = {};
const shapes = {};
const constraints = {};
const matrices = {};
const indexes = {};
const ptrToIndex = {};

const messageQueue = [];

let simulationRate;

let stepDuration = 0;

let freeIndex = 0;

let freeIndexArray;

let world, headerIntArray, headerFloatArray, objectMatricesFloatArray, objectMatricesIntArray, lastTick, getPointer;
let usingSharedArrayBuffer = false;

function isBufferConsumed() {
  if (usingSharedArrayBuffer) {
    return headerIntArray && Atomics.load(headerIntArray, 0) !== BUFFER_STATE.READY;
  } else {
    return objectMatricesFloatArray && objectMatricesFloatArray.buffer.byteLength !== 0;
  }
}

function releaseBuffer() {
  if (usingSharedArrayBuffer) {
    headerFloatArray[1] = stepDuration;
    Atomics.store(headerIntArray, 0, BUFFER_STATE.READY);
  } else {
    postMessage({ type: MESSAGE_TYPES.TRANSFER_DATA, objectMatricesFloatArray, stepDuration }, [
      objectMatricesFloatArray.buffer
    ]);
  }
}

const tick = () => {
  setTimeout(tick, simulationRate);

  if (isBufferConsumed()) {
    const now = performance.now();
    const dt = now - lastTick;
    world.step(dt / 1000);
    stepDuration = performance.now() - now;
    lastTick = now;

    while (messageQueue.length > 0) {
      const message = messageQueue.shift();
      switch (message.type) {
        case MESSAGE_TYPES.ADD_BODY:
          addBody(message);
          break;
        case MESSAGE_TYPES.UPDATE_BODY:
          updateBody(message);
          break;
        case MESSAGE_TYPES.REMOVE_BODY:
          removeBody(message);
          break;
        case MESSAGE_TYPES.ADD_SHAPES:
          addShapes(message);
          break;
        case MESSAGE_TYPES.REMOVE_SHAPES:
          const bodyUuid = message.bodyUuid;
          const shapesUuid = message.shapesUuid;
          if (bodies[bodyUuid] && shapes[shapesUuid]) {
            for (let i = 0; i < shapes[shapesUuid].length; i++) {
              const shape = shapes[shapesUuid][i];
              bodies[bodyUuid].removeShape(shape);
            }
          }
          break;
        case MESSAGE_TYPES.ADD_CONSTRAINT:
          addConstraint(message);
          break;
        case MESSAGE_TYPES.REMOVE_CONSTRAINT:
          const constraintId = message.constraintId;
          if (constraints[constraintId]) {
            constraints[constraintId].destroy();
            delete constraints[constraintId];
          }
          break;
        case MESSAGE_TYPES.UPDATE_CONSTRAINT:
            updateConstraint(message);
            break;
        case MESSAGE_TYPES.ENABLE_DEBUG:
          const enable = message.enable;
          if (!world.debugDrawer) {
            initDebug(message.debugSharedArrayBuffer, world);
          }

          if (world.debugDrawer) {
            if (enable) {
              world.debugDrawer.enable();
            } else {
              world.debugDrawer.disable();
            }
          }
          break;
        case MESSAGE_TYPES.RESET_DYNAMIC_BODY:
          resetDynamicBody(message);
          break;
        case MESSAGE_TYPES.ACTIVATE_BODY:
          activateBody(message);
          break;
        case MESSAGE_TYPES.DEACTIVATE_BODY:
          deactivateBody(message);
          break;
        case MESSAGE_TYPES.SET_LINEAR_VELOCITY:
          setLinearVelocity(message);
          break;
        case MESSAGE_TYPES.SET_ANGULAR_VELOCITY:
          setAngularVelocity(message);
          break;
        case MESSAGE_TYPES.APPLY_FORCE:
          applyForceToBody(message);
          break;
        case MESSAGE_TYPES.APPLY_IMPULSE:
          applyImpulseToBody(message);
          break;
        case MESSAGE_TYPES.APPLY_TORQUE:
          applyTorque(message);
          break;
        default:
          console.error("Unknown message in queue", message);
          break;
      }
    }

    /** Buffer Schema
     * Every physics body has 26 * 4 bytes (64bit float/int) assigned in the buffer
     * 0-15:  Matrix4 elements (floats)
     * 16:    Linear Velocity (float)
     * 17:    Angular Velocity (float)
     * 18-25: first 8 Collisions (ints)
     */

    for (let i = 0; i < uuids.length; i++) {
      const uuid = uuids[i];
      const body = bodies[uuid];
      const index = indexes[uuid];
      const matrix = matrices[uuid];

      matrix.fromArray(objectMatricesFloatArray, index * BUFFER_CONFIG.BODY_DATA_SIZE + BUFFER_CONFIG.MATRIX_OFFSET);
      body.updateShapes();

      if (body.type === TYPE.DYNAMIC) {
        body.syncFromPhysics();
      } else {
        body.syncToPhysics(false);
      }

      objectMatricesFloatArray.set(matrix.elements, index * BUFFER_CONFIG.BODY_DATA_SIZE + BUFFER_CONFIG.MATRIX_OFFSET);

      objectMatricesFloatArray[
        index * BUFFER_CONFIG.BODY_DATA_SIZE + BUFFER_CONFIG.LINEAR_VELOCITY_OFFSET
      ] = body.physicsBody.getLinearVelocity().length();
      objectMatricesFloatArray[
        index * BUFFER_CONFIG.BODY_DATA_SIZE + BUFFER_CONFIG.ANGULAR_VELOCITY_OFFSET
      ] = body.physicsBody.getAngularVelocity().length();

      const ptr = getPointer(body.physicsBody);
      const collisions = world.collisions.get(ptr);
      for (let j = 0; j < BUFFER_CONFIG.BODY_DATA_SIZE - BUFFER_CONFIG.COLLISIONS_OFFSET; j++) {
        if (!collisions || j >= collisions.length) {
          objectMatricesIntArray[index * BUFFER_CONFIG.BODY_DATA_SIZE + BUFFER_CONFIG.COLLISIONS_OFFSET + j] = -1;
        } else {
          const collidingPtr = collisions[j];
          if (ptrToIndex[collidingPtr]) {
            objectMatricesIntArray[index * BUFFER_CONFIG.BODY_DATA_SIZE + BUFFER_CONFIG.COLLISIONS_OFFSET + j] =
              ptrToIndex[collidingPtr];
          }
        }
      }
    }

    releaseBuffer();
  }
};
const initSharedArrayBuffer = (sharedArrayBuffer, maxBodies) => {
  /** BUFFER HEADER
   * When using SAB, the first 4 bytes (1 int) are reserved for signaling BUFFER_STATE
   * This is used to determine which thread is currently allowed to modify the SAB.
   * The second 4 bytes (1 float) is used for storing stepDuration for stats.
   */
  usingSharedArrayBuffer = true;
  headerIntArray = new Int32Array(sharedArrayBuffer, 0, BUFFER_CONFIG.HEADER_LENGTH);
  headerFloatArray = new Float32Array(sharedArrayBuffer, 0, BUFFER_CONFIG.HEADER_LENGTH);
  objectMatricesFloatArray = new Float32Array(
    sharedArrayBuffer,
    BUFFER_CONFIG.HEADER_LENGTH * 4,
    BUFFER_CONFIG.BODY_DATA_SIZE * maxBodies
  );
  objectMatricesIntArray = new Int32Array(
    sharedArrayBuffer,
    BUFFER_CONFIG.HEADER_LENGTH * 4,
    BUFFER_CONFIG.BODY_DATA_SIZE * maxBodies
  );
};

const initTransferrables = arrayBuffer => {
  objectMatricesFloatArray = new Float32Array(arrayBuffer);
  objectMatricesIntArray = new Int32Array(arrayBuffer);
};

function initDebug(debugSharedArrayBuffer, world) {
  const debugIndexArray = new Uint32Array(debugSharedArrayBuffer, 0, 1);
  const debugVerticesArray = new Float32Array(debugSharedArrayBuffer, 4, DefaultBufferSize);
  const debugColorsArray = new Float32Array(debugSharedArrayBuffer, 4 + DefaultBufferSize, DefaultBufferSize);
  world.getDebugDrawer(debugIndexArray, debugVerticesArray, debugColorsArray);
}

function addBody({ uuid, matrix, options }) {
  if (freeIndex !== -1) {
    const nextFreeIndex = freeIndexArray[freeIndex];
    freeIndexArray[freeIndex] = -1;

    indexes[uuid] = freeIndex;
    uuids.push(uuid);
    const transform = new THREE.Matrix4();
    transform.fromArray(matrix);
    matrices[uuid] = transform;

    objectMatricesFloatArray.set(transform.elements, freeIndex * BUFFER_CONFIG.BODY_DATA_SIZE);
    bodies[uuid] = new Body(options || {}, transform, world);
    const ptr = getPointer(bodies[uuid].physicsBody);
    ptrToIndex[ptr] = freeIndex;

    postMessage({ type: MESSAGE_TYPES.BODY_READY, uuid, index: freeIndex });
    freeIndex = nextFreeIndex;
  }
}

function updateBody({ uuid, options }) {
  if (bodies[uuid]) {
    bodies[uuid].update(options);
    bodies[uuid].physicsBody.activate(true);
  }
}

function removeBody({ uuid }) {
  delete ptrToIndex[getPointer(bodies[uuid].physicsBody)];
  bodies[uuid].destroy();
  delete bodies[uuid];
  delete matrices[uuid];
  delete shapes[uuid];
  const index = indexes[uuid];
  freeIndexArray[index] = freeIndex;
  freeIndex = index;
  delete indexes[uuid];
  uuids.splice(uuids.indexOf(uuid), 1);
}

const IDENTITY_MATRIX = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];

function addShapes({ bodyUuid, shapesUuid, vertices, matrices, indexes, matrixWorld, options }) {
  if (!bodies[bodyUuid]) return;

  if (!matrixWorld) {
    matrixWorld = IDENTITY_MATRIX;
  }

  const physicsShapes = createCollisionShapes(vertices, matrices, indexes, matrixWorld, options || { type: SHAPE.BOX });
  for (let i = 0; i < physicsShapes.length; i++) {
    const shape = physicsShapes[i];
    bodies[bodyUuid].addShape(shape);
  }
  shapes[shapesUuid] = physicsShapes;
}

function addConstraint({ constraintId, bodyUuid, targetUuid, options }) {
  if (bodies[bodyUuid] && bodies[targetUuid]) {
    options = options || {};
    if (!options.hasOwnProperty("type")) {
      options.type = CONSTRAINT.LOCK;
    }
    const constraint = new Constraint(options, bodies[bodyUuid], bodies[targetUuid], world);
    constraints[constraintId] = constraint;
  }
}

function updateConstraint({ constraintId, options }) {
  if (constraints[constraintId] != undefined) {
    options = options || {};
    console.log("ammo worker updating constraint! options: " + JSON.stringify(options));
    constraints[constraintId].update(options); //Hm, is there an advantage to calling this instead of
    //just refering to the .physicsConstraint directly?
    /*
    if (options.motorVelocity != undefined) {
      constraints[constraintId].physicsConstraint.setMotorTargetVelocity(options.motorVelocity);
    }
    if (options.motorImpulse != undefined) {
      constraints[constraintId].physicsConstraint.setMaxMotorImpulse(options.motorImpulse);
    }
    if (options.limitHigh != undefined) {
      let limitHigh = options.limitHigh;
      let limitLow = limitHigh * -1;
      if (options.limitLow != undefined) {
        limitLow = options.limitLow;
      }
      constraints[constraintId].physicsConstraint.setLimit(limitLow,limitHigh);
    }
    */
  }
}

function resetDynamicBody({ uuid }) {
  if (bodies[uuid]) {
    const body = bodies[uuid];
    const index = indexes[uuid];
    matrices[uuid].fromArray(objectMatricesFloatArray, index * BUFFER_CONFIG.BODY_DATA_SIZE);
    body.syncToPhysics(true);
    body.physicsBody.getLinearVelocity().setValue(0, 0, 0);
    body.physicsBody.getAngularVelocity().setValue(0, 0, 0);
  }
}

function activateBody({ uuid }) {
  if (bodies[uuid]) {
    bodies[uuid].physicsBody.activate();
  }
}

function deactivateBody({ uuid }) {
  if (bodies[uuid]) {
    bodies[uuid].physicsBody.activate(false);//Maybe??
  }
}

function setLinearVelocity({ uuid, vec }) {
  if (bodies[uuid]) {
    bodies[uuid].physicsBody.getLinearVelocity().setValue(vec.x,vec.y,vec.z);
  }
}

function setAngularVelocity({ uuid, vec }) {
  if (bodies[uuid]) {
    bodies[uuid].physicsBody.getAngularVelocity().setValue(vec.x,vec.y,vec.z);
  }
}

function applyTorque({ uuid, vec }) {
  
  if (bodies[uuid]) {
    //bodies[uuid].physicsBody.applyLocalTorque(vec);
    //bodies[uuid].physicsBody.applyTorque(vec);
    bodies[uuid].physicsBody.applyTorqueImpulse(vec);
  }
}

function applyForceToBody({ uuid, vec }) {
  if (bodies[uuid]) {
    //bodies[uuid].physicsBody.getLinearVelocity().setValue(vec.x,vec.y,vec.z);
    //console.log("trying to apply force to body! ammo.worker!!  " + vec.x.toString() + " " + vec.y.toString() + " " + vec.z.toString());
    //let impulseVec = new THREE.Vector3(0,50,0);
    //let posVec = new THREE.Vector3(0,0,0);
    //let tbv30 = new THREE.Vector3(x_var,1000,z_var);
    //tbv30.setValue(x_var,1000,z_var);
    //let btVec = new Ammo.btVector3(0, 0, 0);
    //console.log("Over here in ammo worker, trying to apply force!!! " + JSON.stringify(vec));
    //bodies[uuid].doSomethingNew();
    //let total = bodies[uuid].physicsBody.getTotalForce();
    //console.log("Total force: " + total.x() + " " + total.y() + " " + total.z());
    bodies[uuid].applyForceAtCenter(vec);
    //bodies[uuid].physicsBody.applyForce(new THREE.Vector3(vec.x,vec.y,vec.z),new THREE.Vector3(0,0,0));
    //bodies[uuid].physicsBody.applyTorque(vec);
    //world.updateBody(bodies[uuid].physicsBody);
    //bodies[uuid].physicsBody.applyForce(vec,new THREE.Vector3(0,0,0));
    //bodies[uuid].physicsBody.applyForce(new Ammo.btVector3(vec.x,vec.y,vec.z));
    //bodies[uuid].physicsBody.applyForce(new THREE.Vector3(vec.x,vec.y,vec.z),new THREE.Vector3(0,0,0));
    //bodies[uuid].physicsBody.applyForce([vec.x,vec.y,vec.z],[0,0,0]);
  }
}

function applyImpulseToBody({ uuid, vec }) {
  if (bodies[uuid]) {
    //bodies[uuid].physicsBody.getLinearVelocity().setValue(vec.x,vec.y,vec.z);
    //console.log("trying to apply impulse to body! ammo.worker!!  " + vec.x.toString() + " " + vec.y.toString() + " " + vec.z.toString());// + val.toString() );
    //let impulseVec = new THREE.Vector3(0,50,0);
    //let posVec = new THREE.Vector3(0,0,0); //NOPE: needs world position
    //let tbv30 = new THREE.Vector3(x_var,1000,z_var);
    //tbv30.setValue(x_var,1000,z_var);
    //bodies[uuid].physicsBody.applyImpulse(impulseVec,posVec);//impulseVec
    let btVec = new Ammo.btVector3(vec.x,vec.y,vec.z);
    bodies[uuid].physicsBody.applyCentralImpulse(btVec);
    //bodies[uuid].physicsBody.applyCentralImpulse(new THREE.Vector3(vec.x,vec.y,vec.z));//Why not both?
  }
}

onmessage = async event => {
  if (event.data.type === MESSAGE_TYPES.INIT) {
    const AmmoModule = initializeWasm(event.data.wasmUrl);

    AmmoModule().then(Ammo => {
      getPointer = Ammo.getPointer;

      const maxBodies = event.data.maxBodies ? event.data.maxBodies : BUFFER_CONFIG.MAX_BODIES;

      freeIndexArray = new Int32Array(maxBodies);
      for (let i = 0; i < maxBodies - 1; i++) {
        freeIndexArray[i] = i + 1;
      }
      freeIndexArray[maxBodies - 1] = -1;

      if (event.data.sharedArrayBuffer) {
        initSharedArrayBuffer(event.data.sharedArrayBuffer, maxBodies);
      } else if (event.data.arrayBuffer) {
        initTransferrables(event.data.arrayBuffer);
      } else {
        console.error("A valid ArrayBuffer or SharedArrayBuffer is required.");
      }

      world = new World(event.data.worldConfig || {});
      lastTick = performance.now();
      simulationRate = event.data.simulationRate === undefined ? CONSTANTS.SIMULATION_RATE : event.data.simulationRate;
      self.setTimeout(tick, simulationRate);
      postMessage({ type: MESSAGE_TYPES.READY });
    });
  } else if (event.data.type === MESSAGE_TYPES.TRANSFER_DATA) {
    if (event.data.simulationRate !== undefined) {
      simulationRate = event.data.simulationRate;
    }
    objectMatricesFloatArray = event.data.objectMatricesFloatArray;
    objectMatricesIntArray = new Int32Array(objectMatricesFloatArray.buffer);
  } else {
    messageQueue.push(event.data);
  }
};
