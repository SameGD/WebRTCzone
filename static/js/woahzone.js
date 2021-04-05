/*
    woahzone.js
    a multiplayer vibe experiment by Riley Taylor (rtay.io)
*/

/// --- IMPORTS --- ///
import * as THREE from './three.module.js';
import {GLTFLoader} from './loaders/GLTFLoader.js';
import {DRACOLoader} from './loaders/DRACOLoader.js';
import {PointerLockControls} from './controls/PointerLockControls.js';

/// --- SOCKET CONSTANTS --- ///
let ID;
const SOCKET = io.connect(document.documentURI);
const USERS = {};
const TICKRATE = 15;
const FRAMERATE = 60;

/// --- THREEJS CONSTANTS --- ///
const MANAGER = new THREE.LoadingManager();
const DRACO_LOADER = new DRACOLoader(MANAGER);
const GLTF_LOADER = new GLTFLoader(MANAGER);
const CUBE_TEXTURE_LOADER = new THREE.CubeTextureLoader();
const FONT_LOADER = new THREE.FontLoader(MANAGER);
const FONT_SIZE = 0.08;

const CANVAS_HOLDER = document.getElementById('canvas-holder');

const CAMERA = new THREE.PerspectiveCamera(65, window.innerWidth / window.innerHeight, 0.02, 60);
const SCENE = new THREE.Scene();
const RENDERER = new THREE.WebGLRenderer({antialias:true, powerPreference:"high-performance", stencil:false, alpha:true, depth:true, precision:"lowp"});
const CONTROLS = new PointerLockControls(CAMERA, document.body);
const PLAYER = CONTROLS.getObject();

const TRANSPARENT_MATERIALS = ['tallgrass', 'vines'];
const PLAYER_MODELS = {};

const DIR = {FORWARD:0, BACKWARD:1, LEFT:2, RIGHT:3, UP:4, DOWN:5, SPRINT:6};
const PLAYER_MOVE = [0, 0, 0, 0, 0, 0, 0];
const SPEED_NORMAL = 8;
const SPEED_SPRINT = 14;

/// --- VARIABLES --- ///
var name = undefined;
var model = undefined;

var time, delta, moveTimer = 0;
var useDeltaTiming = true, weirdTiming = 0;
var prevTime = performance.now();

var openSansFont;

/// --- WEB-RTC --- ///

// Video / Stream Vars

const constraints = {
  audio: true,
  video: {
    width: { ideal: 1280 },
    height: { ideal: 720 }
  }
};

const peerList = [];
let stream;

// Configures the addresses of the ICE servers that this code will be using. The ICE servers get around port forwarding / network fuckery by outsourcing the determination of each peers network location to chumps like Google. (Or something like that, idk I'm a scientist, not a network engineer).

const config = {
  iceServers: [{
    urls: [
      "stun:stun.l.google.com:19302",
      "stun:stun2.l.google.com:19302",
    ]
  }]
};


// This demo depends on the canvas element
if (!('getContext' in document.createElement('canvas'))) {
    alert('Sorry, it looks like your browser does not support canvas!');
}

// Also make sure webgl is enabled on the current machine
if (WEBGL.isWebGLAvailable()) {
    // If everything is possible, start the app, otherwise show an error
    // Gets the usermedia before joining the server, so other clients have a stream they can request
    init();
    gameLoop();
} else {
    let warning = WEBGL.getWebGLErrorMessage();
    document.body.appendChild(warning);
    CANVAS_HOLDER.remove();
    throw 'WebGL disabled or not supported';
}


///// ----- ASYNC FUNCTIONS ----- /////
// Establish connection
SOCKET.on('connect', () => {
    console.log("Connection established to server");

    // This was randomly generated on the client end, but I changed it to be the socket ID to better mesh with my WebRTC code
    ID = SOCKET.id;

    console.log("DANE: ID is " + ID);

    // Initialises WebRTC by getting the users local video and audio stream.
    // This is done before the server is joined, as Peer Connections are made upon someone else joining the server, and it'll throw an error if theres no stream to send to the other person.
    initWebRTC();

    // Broadcast join to other users
    SOCKET.emit('join', {id: ID});

    // Unblur screen and give control access
    CANVAS_HOLDER.style.filter = "blur(0px)";
    let blockInputElement = document.getElementById('block-input');
    if (typeof(blockInputElement) !== 'undefined' && blockInputElement !== null) {
        blockInputElement.remove();
    }
});

// Receive assigned identity
SOCKET.on('selfIdentity', (data) => {
    name = data.name;
    model = data.model;
    console.log("You are a(n) " + model + " with the name " + name);
});

/// Handle information from other users
SOCKET.on('otherJoin', (data) => {
    console.log(data.name, "has joined the server");

    // Load the player data

    USERS[data.id] = {
        'name': data.name,
        'pos': new THREE.Vector3(0,0,0),
        'rot': new THREE.Quaternion(0, 0, 0, 0),
        'oldPos': new THREE.Vector3(0,0,0),
        'alpha': 0
    }

    // Start the WebRTC connection process

    createPeerConnection(data.id);

    // Send identity back if you have it
    if (name !== "" && name !== undefined) {
        emitIdentity(data.id);
        emitMove();
    }
});

SOCKET.on('otherIdentity', (data) => {
    if (USERS[data.id] !== undefined) {
        USERS[data.id].name = data.name;
        USERS[data.id].model = data.model;
    } else {
        console.log(data.name, "is already on the server");

        // Haven't met this player before, so create them on our end

        USERS[data.id] = {
            'name': data.name,
            'pos': new THREE.Vector3(0,0,0),
            'rot': new THREE.Quaternion(0, 0, 0, 0),
            'oldPos': new THREE.Vector3(0,0,0),
            'alpha': 0
        }


    }
});

SOCKET.on('otherMove', (data) => {
    let userid = data.id;
    if (userid in USERS) {
        if (USERS[userid] !== undefined) {
            if (USERS[userid].mesh !== undefined) {
                USERS[userid].oldPos.copy(USERS[userid].mesh.position);
            }
            USERS[userid].pos.set(data.pos.x, data.pos.y, data.pos.z);
            USERS[userid].rot.set(data.rot.x, data.rot.y, data.rot.z, data.rot.w);
            USERS[userid].alpha = 0;

            let userVideo = document.getElementById(userid);

            if (userVideo !== null) {
              let distance = PLAYER.position.distanceTo(data.pos);
              userVideo.volume = Math.min(1, 1 / distance);
            }

        }
    }
});

SOCKET.on('otherDisconnect', (userid) => {
    if (USERS[userid] !== undefined) {
        console.log(USERS[userid].name, "has disconnected");
        SCENE.remove(USERS[userid].text);
        SCENE.remove(USERS[userid].mesh);
        USERS[userid] = undefined;

        // Removes the video element that was holding their video stream, and removes them from the dictionary of WebRTC Peers

        document.getElementById(userid).remove();
        delete peerList[userid];

    }
});

// If client is disconnected unexpectedly (i.e. booted from server or server connection lost)
SOCKET.on('disconnect', (data) => {
    // Determine which disconnect has occurred and display relevant error
    switch(data) {
        case("io server disconnect"):
            alert("Kicked from server!");
            break;
        case("ping timeout"):
            alert("Timed out from server.");
            break;
        case("transport close"):
            alert("Lost connection to server.");
            break;
        default:
            alert("Disconnected due to an unknown error.\nPlease reconnect.");
    }
    // Delete the canvas module to prevent user from still moving around after disconnect
    CANVAS_HOLDER.parentNode.removeChild(CANVAS_HOLDER);
});

// Send information about self to others
function emitMove() {
    SOCKET.emit('move', {
        id: ID,
        pos: PLAYER.position,
        rot: {x:PLAYER.quaternion.x, y:PLAYER.quaternion.y, z:PLAYER.quaternion.z, w:PLAYER.quaternion.w}
    });
}

function emitIdentity(target) {
    SOCKET.emit('identity', {
        id: ID,
        name: name,
        model: model,
        target: target
    });
}

///// ----- SYNCRHONOUS FUNCTIONS ----- /////
// ThreeJS initialisation stuff
function init() {

    // Create the loading manager
    initManager()

    // Initialise everything in the scene
    initScene("./mesh/weddingquake.min.glb");
    initSkybox();
    initLights();
    initPlayer();
    initRenderer();
    initFonts();


    // Add a listener event for window resizing
    window.addEventListener('resize', onWindowResize, false);
}

function initManager() {
    MANAGER.onStart = function(managerUrl, itemsLoaded, itemsTotal) {
        //console.log('Started loading: ' + managerUrl + '\nLoaded ' + itemsLoaded + ' of ' + itemsTotal + ' files.');
    };

    MANAGER.onProgress = function(managerUrl, itemsLoaded, itemsTotal) {
        document.getElementById('progress-bar').style.width = (itemsLoaded / itemsTotal * 100) + '%';
        //console.log('Loading file: ' + managerUrl + '.\nLoaded ' + itemsLoaded + ' of ' + itemsTotal + ' files.');
    };

    MANAGER.onLoad = function () {
        // Only allow control once content is fully loaded
        CANVAS_HOLDER.addEventListener('click', function () {
            CONTROLS.lock();
        }, false);

        console.log('Loading complete!');
        document.getElementById('progress').hidden = true;
    };
}

function initScene(sceneFile) {
    // Create a new scene
    SCENE.background = new THREE.Color(0x000000);
    //SCENE.overrideMaterial = new THREE.MeshLambertMaterial();  <--- Funny gamer mode

    // Use dracoloader for decompression
    DRACO_LOADER.setDecoderPath('./js/loaders/draco/');
    DRACO_LOADER.setDecoderConfig({ type: 'js' });

    // Load the scene geometry
    GLTF_LOADER.setDRACOLoader(DRACO_LOADER);
    GLTF_LOADER.load(
        sceneFile,
        (gltf) => {
            // Add the level to the scene
            let root = gltf.scene;
            processMaterials(root);
            SCENE.add(root);
            SCENE.matrixAutoUpdate = false;
        },
        (xhr) => {
            document.getElementById('progress-bar').style.width = (xhr.loaded / xhr.total * 100) + '%';
        },
        (error) => {
            console.log('Error loading', sceneFile);
            console.log(error);
        }
    );
}

function initSkybox() {
    // Load skybox
    let skyboxArray = ["ft", "bk", "up", "dn", "rt", "lf"];
    for (let i in skyboxArray) {
        skyboxArray[i] = "./img/skybox/hell_" + skyboxArray[i] + ".min.png";
    }
    CUBE_TEXTURE_LOADER.load(skyboxArray, (texture) => {
        texture.encoding = THREE.sRGBEncoding;
        texture.magFilter = THREE.NearestFilter;
        texture.minFilter = THREE.NearestFilter;
        SCENE.background = texture;
    });
}

function initLights() {
    // A single huge hemisphere light for global shading
    let light = new THREE.HemisphereLight(0xffffcc, 0x331111, 1);
    light.position.set(50, 100, 0);
    SCENE.add(light);
}

function initPlayer() {
    PLAYER.position.fromArray([0, 0, 0]);
    PLAYER.speedMultiplier = 1
    SCENE.add(PLAYER);

    document.addEventListener('keydown', function (event) {
        switch (event.code) {
            case "ArrowUp":
            case "KeyW":
                PLAYER_MOVE[DIR.FORWARD] = true;
                break;
            case "ArrowLeft":
            case "KeyA":
                PLAYER_MOVE[DIR.LEFT] = true;
                break;
            case "ArrowDown":
            case "KeyS":
                PLAYER_MOVE[DIR.BACKWARD] = true;
                break;
            case "ArrowRight":
            case "KeyD":
                PLAYER_MOVE[DIR.RIGHT] = true;
                break;
            case "KeyE":
            case "PageUp":
            case "Space":
                PLAYER_MOVE[DIR.UP] = true;
                break;
            case "KeyQ":
            case "PageDown":
                PLAYER_MOVE[DIR.DOWN] = true;
                break;
            case "ShiftLeft":
                PLAYER_MOVE[DIR.SPRINT] = true;
                break;
        }
    }, false);

    document.addEventListener('keyup', function (event) {
        switch (event.code) {
            case "ArrowUp":
            case "KeyW":
                PLAYER_MOVE[DIR.FORWARD] = false;
                break;
            case "ArrowLeft":
            case "KeyA":
                PLAYER_MOVE[DIR.LEFT] = false;
                break;
            case "ArrowDown":
            case "KeyS":
                PLAYER_MOVE[DIR.BACKWARD] = false;
                break;
            case "ArrowRight":
            case "KeyD":
                PLAYER_MOVE[DIR.RIGHT] = false;
                break;
            case "KeyE":
            case "PageUp":
            case "Space":
                PLAYER_MOVE[DIR.UP] = false;
                break;
            case "KeyQ":
            case "PageDown":
                PLAYER_MOVE[DIR.DOWN] = false;
                break;
            case "ShiftLeft":
                PLAYER_MOVE[DIR.SPRINT] = false;
                break;
        }
    }, false);
}

function initRenderer() {
    RENDERER.setPixelRatio(0.7);
    RENDERER.setSize(window.innerWidth, window.innerHeight);
    RENDERER.outputEncoding = THREE.GammaEncoding;
    RENDERER.gammaFactor = 2.2;
    CANVAS_HOLDER.appendChild(RENDERER.domElement);
}

function initFonts() {
    FONT_LOADER.load('./font/OpenSans_Regular.json',
        (font) => {
            openSansFont = font;

            // If there are any users already in the server before the font loaded, generate text for them
            for (let userid in USERS) {
                if (USERS[userid].text === undefined) {
                    let textMesh = createTextMesh(USERS[userid].name, FONT_SIZE);
                    USERS[userid].text = textMesh;
                    SCENE.add(textMesh);
                }
            }
        }
    );
}

// ThreeJS main game/render loop
function gameLoop() {
    requestAnimationFrame(gameLoop);

    time = performance.now();
    if (useDeltaTiming) {
        delta = (time - prevTime) / 1000;
        // some code that checks if timing is weird and then turns off delta-timing
        // this is specifically for those people running this in firefox with privacy.resistFingerprinting enabled
        if (delta === 0.1) {
            weirdTiming += 1;
            if (weirdTiming === 5) {
                useDeltaTiming = false;
                console.warn("HUMAN.Riley: performance.now() warning: The performance API in your browser is returning strange time measurements, perhaps due to a privacy or anti-fingerprinting setting you've enabled. This may affect your performance :(")
            }
        }
    } else {
        delta = 1/FRAMERATE;
    }

    // Process player input
    if (CONTROLS.isLocked) {
        // Sprinting
        let moveSpd;
        if (PLAYER_MOVE[DIR.SPRINT]) {
            moveSpd = SPEED_SPRINT * PLAYER.speedMultiplier;
        } else {
            moveSpd = SPEED_NORMAL * PLAYER.speedMultiplier;
        }

        // Move forwards/backwards
        let dolly = PLAYER_MOVE[DIR.BACKWARD] - PLAYER_MOVE[DIR.FORWARD];
        if (dolly !== 0) {
            PLAYER.translateZ(dolly * moveSpd * delta);
        }

        // Move left/right
        let strafe = PLAYER_MOVE[DIR.RIGHT] - PLAYER_MOVE[DIR.LEFT];
        if (strafe !== 0) {
            PLAYER.translateX(strafe * moveSpd * delta);
        }

        // Move up/down
        let heave = PLAYER_MOVE[DIR.UP] - PLAYER_MOVE[DIR.DOWN];
        if (heave !== 0) {
            PLAYER.position.y += (heave * moveSpd * delta);
        }
    }

    // Broadcast movement to other players n times per second
    moveTimer += delta;
    if (moveTimer >= 1/TICKRATE) {
        moveTimer = 0;
        emitMove();
    }

    // Move other players (interpolate movement)
    for (let userid in USERS) {
        if (USERS[userid] !== undefined) {
            let oldPos = USERS[userid].oldPos;
            let pos = USERS[userid].pos;
            let rot = USERS[userid].rot;
            let a = USERS[userid].alpha;

            if (USERS[userid].mesh !== undefined) {
                USERS[userid].mesh.position.lerpVectors(oldPos, pos, a);
                USERS[userid].mesh.quaternion.rotateTowards(rot, USERS[userid].mesh.quaternion.angleTo(rot) * (TICKRATE * delta));
                if (USERS[userid].text !== undefined) {
                    USERS[userid].text.position.copy(USERS[userid].mesh.position);
                    USERS[userid].text.rotation.copy(USERS[userid].mesh.rotation);
                }
            }

            USERS[userid].alpha = Math.min(a + delta*(TICKRATE-1), 2);
        }
    }

    prevTime = time;
    RENDERER.render(SCENE, CAMERA);
}

// Loading functions
function loadPlayerModel(model) {
    // Load a specific player model into the scene
    GLTF_LOADER.load(
        './mesh/playermodels/' + model + '.min.glb',
        (gltf) => {
            // Once model has been downloaded, scale it appropriately and load it into memory
            let modelScene = gltf.scene;
            processMaterials(modelScene);
            PLAYER_MODELS[model] = modelScene;

            // Also create an instance of the model for all players with that model
            for (let userid in USERS) {
                if (USERS[userid] !== undefined) {
                    if (USERS[userid].model === model) {
                        USERS[userid].mesh = PLAYER_MODELS[model].clone();
                        SCENE.add(USERS[userid].mesh);
                    }
                }
            }
        }
    );

}

function createOtherPlayer(peerId, video) {

    // Load 3D model
    // if (model in PLAYER_MODELS && PLAYER_MODELS[model] !== undefined) {
    //     // If it's already loaded, assign it to the user
    //     USERS[userid].mesh = PLAYER_MODELS[model].clone();
    //     SCENE.add(USERS[userid].mesh);
    // } else if (!(model in PLAYER_MODELS)) {
    //     // If it's not loaded, and not being loaded, then load it into the scene
    //     // loadPlayerModel() will automatically assign it to the user when the mesh is loaded
    //     PLAYER_MODELS[model] = undefined;
    //     loadPlayerModel(model);
    // }

  	let videoTexture = new THREE.VideoTexture(video);
    //videoTexture.minFilter = THREE.LinearFilter;
    videoTexture.format = THREE.RGBFormat;

    let materials = [
      new THREE.MeshLambertMaterial({color:0x888888}),
      new THREE.MeshLambertMaterial({color:0x888888}),
      new THREE.MeshLambertMaterial({color:0x888888}),
      new THREE.MeshLambertMaterial({color:0x888888}),
      new THREE.MeshLambertMaterial({color:0x888888}),
      new THREE.MeshLambertMaterial( {color:0xffffff, map: videoTexture, side: THREE.DoubleSide } ),
    ]

  	// the geometry on which the movie will be displayed;
  	let videoGeometry = new THREE.CubeGeometry(1.5, 1, 0.5);
    // Flips the screen horizontally
    videoGeometry.scale.x = -1;
  	// attach video to a mesh that will move with the camera
  	let videoScreen = new THREE.Mesh(videoGeometry, materials);

    processMaterials(videoScreen)

    USERS[peerId].mesh = videoScreen;

    SCENE.add(USERS[peerId].mesh);

    // Add text above the user's head if the font has loaded
    // If the font hasn't loaded yet, it will automatically add text above all current user's heads when it loads
    if (openSansFont !== undefined) {
        // Create the text mesh and assign it to the user
        let textMesh = createTextMesh(USERS[peerId].name, FONT_SIZE);
        USERS[peerId].text = textMesh;
        SCENE.add(textMesh);
    }
}

function createTextMesh(message, fontSize) {
    let textMat, textShapes, textGeometry, textXOffset, textMesh;

    // Create the text geometry and material
    textMat = new THREE.MeshBasicMaterial({color: 0x000000, side: THREE.DoubleSide})
    textShapes = openSansFont.generateShapes(message, fontSize);
    textGeometry = new THREE.ShapeBufferGeometry(textShapes);
    textGeometry.computeBoundingBox();

    // Center align text
    textXOffset = -0.5 * (textGeometry.boundingBox.max.x - textGeometry.boundingBox.min.x);
    textGeometry.translate(textXOffset, 0.8, 0);
    textGeometry.rotateY(Math.PI);

    // Generate text mesh
    textMesh = new THREE.Mesh(textGeometry, textMat);
    return textMesh;
}

// Miscellaneous functions
function processMaterials(obj) {
    // Recursively goes through all materials and modifies them so the scene is displayed correctly
    //  - Enables backface culling on all materials
    //  - Enables alpha testing for some materials

    obj.children.forEach((child) => {
        if (child.hasOwnProperty("material")) {

            // Enable backface culling
            child.material.side = THREE.FrontSide;

            // Improve the mipmaps
            if (child.material.map != null) {
                child.material.map.magFilter = THREE.NearestFilter;
                child.material.map.minFilter = THREE.NearestMipmapNearestFilter;//THREE.LinearMipmapNearestFilter;//
                child.material.map.anisotropy = 1;
            }

            // Enable alpha testing if the material is tagged as transparent
            if (TRANSPARENT_MATERIALS.indexOf(child.material.name) > -1) {
                child.material.alphaTest = 0.5;
            } else {
                child.material.alphaTest = 1;
            }

            child.material.transparent = false;
            child.material.color.convertSRGBToLinear();

            // TODO: Materials could be converted into MeshLambertMaterial instead of MeshStandardMaterial for optimisation, would SCENE.overrideMaterial work?
        }
        processMaterials(child);
    });
}

function onWindowResize() {
    CAMERA.aspect = window.innerWidth / window.innerHeight;
    CAMERA.updateProjectionMatrix();
    RENDERER.setSize(window.innerWidth, window.innerHeight);
}

/// --- WebRTC Code (Comprehend at own risk!) --- ///

function initWebRTC() {

  navigator.mediaDevices.getUserMedia(constraints)
  .then(function(strm) {
    stream = strm;

    peerList.forEach((peerConnection) => {
      stream.getTracks().forEach((track) => peerConnection.addTrack(track, stream));
    });

  })
  .catch(function(err) {
    console.log(err)
  });

}

async function createPeerConnection(peerId) {

  // Checks if Peer Connection has already been made for the

  if (!(peerId in peerList)) {

    // Sets a new peer connection, that will be found with the ice servers declared in the config variable.

    peerList[peerId] = new RTCPeerConnection(config);

    // Adds each stream to the peer connection, for it to be sent later

    if (stream !== undefined) {

      stream.getTracks().forEach((track) => peerList[peerId].addTrack(track, stream));

    }

    // Once the ICE server finds a viable candidate route for the two connections to talk down, an event will be called. This allows the candidate route to be sent over the sockets server to the other client.

    peerList[peerId].onicecandidate = (event) => {
      SOCKET.emit("RTC-request", {sender: SOCKET.id, receiver: peerId, candidate: event.candidate});
      console.log("Sent Ice Candidate to " + USERS[peerId].name);
    };

    // So what I think is happening is that this proposes a connection between the peer and the user. So it sets the description of the proposed connection locally, creates an offer to the other peer, and sends the proposed description that was just sent to the other peer. This probably ensures both connections are on the same page or something.

    peerList[peerId].onnegotiationneeded = async () => {
      try {

        await peerList[peerId].setLocalDescription(await peerList[peerId].createOffer());

        SOCKET.emit("RTC-request", {sender: SOCKET.id, receiver: peerId, desc: peerList[peerId].localDescription});

      } catch (err) {
        console.error(err);
      }
    };

    // It always generates two video streams for some reason, so this stops that from happening

    let hasVideo = false;

    // Once it gets a connection with the peer with a stream attached, it attaches that stream to the video object.

    peerList[peerId].ontrack = async (event) => {

      if (hasVideo == false) {

        console.log("Making Video Element for " + USERS[peerId].name);

        addVideo(event, peerId);
        hasVideo = true;
      };

    };

  }
}

async function addVideo(video, peerId) {

  // Creates the video element

  let remoteView = document.createElement("video");

  // Attaches the video stream

  remoteView.srcObject = video.streams[0];

  // Sets the settings of the video element, and attaches it to the videoDiv

  remoteView.height = "720";
  remoteView.width = "1280";
  remoteView.id = peerId;
  remoteView.autoplay = true;
  remoteView.playsinline = true;
  remoteView.style.visibility = "hidden";

  document.getElementById("videoContainer").appendChild(remoteView);

  // Creates the model for the other player

  createOtherPlayer(peerId, remoteView);

  video.track.onended = (event) => {

    console.log("Stream ended");

    remoteView.remove();
    delete peerList[peerId];

  };
}

SOCKET.on("RTC-request", async ({sender, receiver, desc, candidate}) => {

  try {

    createPeerConnection(sender);

    if (desc) {

      if (desc.type === "offer") {

        console.log("Got Offer from " + USERS[sender].name);

        await peerList[sender].setRemoteDescription(desc);

        await peerList[sender].setLocalDescription(await peerList[sender].createAnswer());

        SOCKET.emit("RTC-request", {sender: SOCKET.id, receiver: sender, desc: peerList[sender].localDescription});

      } else if (desc.type === "answer") {

        console.log("Got Answer from " + USERS[sender].name);

        await peerList[sender].setRemoteDescription(desc);
      }

    } else if (candidate) {

      console.log("Got Candidate from " + USERS[sender].name);

      await peerList[sender].addIceCandidate(candidate);
    }
  } catch (err) {
    console.error(err);
  }

});
