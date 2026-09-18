import * as THREE from 'three';
import { Vector3 } from 'three';

function decodeModelName(model, address) {
    const end = model.names.indexOf(0, address);
    return new TextDecoder('utf-8').decode(model.names.subarray(address, end));
}

/** Return the named body and every body below it in the MuJoCo body tree. */
export function collectBodyTreeIds(model, rootBodyName) {
    const result = new Set();
    if (!model || !rootBodyName) return result;

    let rootBodyId = -1;
    for (let bodyId = 1; bodyId < model.nbody; bodyId++) {
        if (decodeModelName(model, model.name_bodyadr[bodyId]) === rootBodyName) {
            rootBodyId = bodyId;
            break;
        }
    }
    if (rootBodyId < 0) return result;

    for (let bodyId = rootBodyId; bodyId < model.nbody; bodyId++) {
        let ancestorId = bodyId;
        while (ancestorId > 0) {
            if (ancestorId === rootBodyId) {
                result.add(bodyId);
                break;
            }
            ancestorId = model.body_parentid[ancestorId];
        }
    }
    return result;
}

export class DragStateManager {
    constructor(scene, renderer, camera, container, controls) {
        this.scene = scene;
        this.renderer = renderer;
        this.camera = camera;
        this.mousePos = new THREE.Vector2();
        this.raycaster = new THREE.Raycaster();
        //this.raycaster.layers.set(1);
        //					this.raycaster.params.Mesh.threshold = 3;
        this.raycaster.params.Line.threshold = 0.1;
        this.grabDistance = 0.0;
        this.active = false;
        this.physicsObject = null;
        this.controls = controls;
        this.draggableBodyIds = new Set();

        this.arrow = new THREE.ArrowHelper(new THREE.Vector3(0, 1, 0), new THREE.Vector3(0, 0, 0), 15, 0x666666);
        this.arrow.setLength(15, 3, 1);
        this.scene.add(this.arrow);
        //this.residuals.push(arrow);
        this.arrow.line.material.transparent = true;
        this.arrow.cone.material.transparent = true;
        this.arrow.line.material.opacity = 0.5;
        this.arrow.cone.material.opacity = 0.5;
        this.arrow.visible = false;

        this.previouslySelected = null;
        this.higlightColor = 0xff0000;  // 0x777777

        this.localHit = new Vector3();
        this.worldHit = new Vector3();
        this.currentWorld = new Vector3();

        container.addEventListener( 'pointerdown', this.onPointer.bind(this), true );
        document.addEventListener( 'pointermove', this.onPointer.bind(this), true );
        document.addEventListener( 'pointerup'  , this.onPointer.bind(this), true );
        document.addEventListener( 'pointerout' , this.onPointer.bind(this), true );
        container.addEventListener( 'dblclick', this.onPointer.bind(this), false );
    }
    configure(model, rootBodyName) {
        this.draggableBodyIds = collectBodyTreeIds(model, rootBodyName);
        if (this.active && !this.draggableBodyIds.has(this.physicsObject?.bodyID)) this.end();
    }
    updateRaycaster(x, y) {
        var rect = this.renderer.domElement.getBoundingClientRect();
        this.mousePos.x =  ((x - rect.left) / rect.width) * 2 - 1;
        this.mousePos.y = -((y - rect.top) / rect.height) * 2 + 1;
        this.raycaster.setFromCamera(this.mousePos, this.camera);
    }
    start(x, y) {
        this.physicsObject = null;
        this.updateRaycaster(x, y);
        let intersects = this.raycaster.intersectObjects(this.scene.children, true);
        for (let i = 0; i < intersects.length; i++) {
            let obj = intersects[i].object;
            if (this.draggableBodyIds.has(obj.bodyID)) {
                this.physicsObject = obj;
                this.grabDistance = intersects[i].distance;
                let hit = this.raycaster.ray.origin.clone();
                hit.addScaledVector(this.raycaster.ray.direction, this.grabDistance);
                this.arrow.position.copy(hit);
                //this.physicsObject.startGrab(hit);
                this.active = true;
                this.controls.enabled = false;
                this.localHit = obj.worldToLocal(hit.clone());
                this.worldHit.copy(hit);
                this.currentWorld.copy(hit);
                this.arrow.visible = true;
                break;
            }
        }
    }
    move(x, y) {
        if (this.active) {
            this.updateRaycaster(x, y);
            let hit = this.raycaster.ray.origin.clone();
            hit.addScaledVector(this.raycaster.ray.direction, this.grabDistance);
            this.currentWorld.copy(hit);

            this.update();

            if (this.physicsObject != null) {
                //this.physicsObject.moveGrabbed(hit);
            }
        }
    }
    update() {
        if (this.worldHit && this.localHit && this.currentWorld && this.arrow && this.physicsObject) {
            this.worldHit.copy(this.localHit);
            this.physicsObject.localToWorld(this.worldHit);
            this.arrow.position.copy(this.worldHit);
            this.arrow.setDirection(this.currentWorld.clone().sub(this.worldHit).normalize());
            this.arrow.setLength(this.currentWorld.clone().sub(this.worldHit).length());
        }
    }
    end(evt) {
        //this.physicsObject.endGrab();
        this.physicsObject = null;

        this.active = false;
        this.controls.enabled = true;
        //this.controls.onPointerUp(evt);
        this.arrow.visible = false;
        this.mouseDown = false;
    }
    onPointer(evt) {
        if (evt.type == "pointerdown") {
            this.start(evt.clientX, evt.clientY);
            this.mouseDown = true;
        } else if (evt.type == "pointermove" && this.mouseDown) {
            if (this.active) { this.move(evt.clientX, evt.clientY); }
        } else if (evt.type == "pointerup" /*|| evt.type == "pointerout"*/) {
            this.end(evt);
        }
        if (evt.type == "dblclick") {
            this.updateRaycaster(evt.clientX, evt.clientY);
            const selected = this.raycaster.intersectObjects(this.scene.children, true)
                .map((intersection) => intersection.object)
                .find((object) => this.draggableBodyIds.has(object.bodyID)) ?? null;
            this.doubleClick = true;
            if (selected) {
                if (selected == this.previouslySelected) {
                    selected.material.emissive.setHex(0x000000);
                    this.previouslySelected = null;
                } else {
                    if (this.previouslySelected) {
                        this.previouslySelected.material.emissive.setHex(0x000000);
                    }
                    selected.material.emissive.setHex(this.higlightColor);
                    this.previouslySelected = selected;
                }
            } else {
                if (this.previouslySelected) {
                    this.previouslySelected.material.emissive.setHex(0x000000);
                    this.previouslySelected = null;
                }
            }
        }
    }
}
