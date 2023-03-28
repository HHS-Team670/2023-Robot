
// initial camera settings
var driveReversed = false
var allKeysPressed = new Array()
var angle = 0
var selectedPath = ""

var single_substation_coords = []


function updatePath(evt) {
    if (document.querySelector(".path-dropdown:hover") != null) {
        evt.preventDefault()
        selectedPath = evt.target.innerHTML
        let dropupBtn = document.querySelector(".dropbtn")
        dropupBtn.innerHTML = selectedPath
    }
}

function resetAndAddDropdownListeners() {
    document.querySelector(".dropbtn").innerHTML = "Choose Path"
    var paths = document.querySelectorAll(".path-dropdown .dropup-content p")
    for (let i = 0; i < paths.length; i++) {
        let path = paths[i]
        path.onclick = updatePath
    }
}

function setPaths() { 

    var content = document.querySelector(".dropup-content")
    content.innerHTML = ""

    var p1 = document.createElement("p")
    p1.appendChild(document.createTextNode("cableScore"))
    
    var p2 = document.createElement("p")
    p2.appendChild(document.createTextNode("stationScore"))

    var p3 = document.createElement("p")
    p3.appendChild(document.createTextNode("cableEngage"))

    var p4 = document.createElement("p")
    p4.appendChild(document.createTextNode("stationEngage"))

    var p5 = document.createElement("p")
    p5.appendChild(document.createTextNode("centerEngage"))

    var p6 = document.createElement("p")
    p6.appendChild(document.createTextNode("centerIntake"))

    var p7 = document.createElement("p")
    p7.appendChild(document.createTextNode("scoreMid"))

    content.append(p1, p2, p3, p4, p5, p6, p7)
    resetAndAddDropdownListeners()
}

setPaths()


// var toggleCamera = document.querySelector('#toggle-camera')
// toggleCamera.onclick = () => {
//     var cameraDiv = document.querySelector("#camera-streams")
//     if (document.querySelector(".camera-stream") == null) {
//         var cameraStreams = `<image class="camera-stream" src="http://photonvision.local:1182/stream.mjpg?1664842058013"></image>
//         <image class="camera-stream" src="http://10.6.70.2:1181"></image>`
//         // var cameraStreams = `<image class="camera-stream" src="http://photonvision.local:1184/stream.mjpg?1646964457545"></image>`
//         cameraDiv.insertAdjacentHTML( 'beforeend', cameraStreams )
//     } else {
//         while (cameraDiv.firstChild) {
//             cameraDiv.removeChild(cameraDiv.firstChild)
//         }
//     }
// }

// // listens for robot-state and updates status lights and auton chooser accordingly
// NetworkTables.addKeyListener('/SmartDashboard/robot-state', (key, value) => {
//     if (value === "autonomousInit()" || value === "disabledPeriodic()") {
//         document.getElementById('auton-chooser').style.display = "none"
//     } else if (value === "autonomousPeriodic()") {
//         document.getElementById('auton-status').style.fill = "rgb(0,255,0)"
//         document.getElementById('auton-status').style.stroke = "rgb(0,255,0)"
//     } else if (value === "teleopInit()" || value === "teleopPeriodic()") {
//         document.getElementById('auton-status').style.fill = "none"
//         document.getElementById('auton-status').style.stroke = "rgb(255,255,255)"
//     }
// })

// // listens for warnings
// NetworkTables.addKeyListener('/SmartDashboard/warnings', (key, value) => {
//     // document.getElementById('big-warning').style.display = "inline"
//     // document.getElementById('warnings').innerHTML += (value + "\n")

//     // setTimeout(() => { document.getElementById('big-warning').style.display = "none" }, 1000)
//     // var timeSinceWarningFlashed = Date.getTime()
//     console.log(value)
// })



NetworkTables.addKeyListener('/SmartDashboard/match-started', (key, value) => {
    var autoSelector = document.querySelector('#auton-chooser')
    if (value) {
        autoSelector.style.display = 'none'
    } else {
        autoSelector.style.display = 'block'
    }
})


// updates status lights for driveBase
NetworkTables.addKeyListener('/SmartDashboard/aligned', (key, value) => {
    // MOVE_UP,
    //    MOVE_DOWN,
    //    MOVE_LEFT,
    //    MOVE_RIGHT,
    //    TURN_CLOCK,
    //    TURN_COUNTERCLOCK,
    //    OKAY

    
    var moveStatus = value[0]
    var strafeStatus = value[1]
    var turnStatus = value[2]
    var okay = true

    var body = document.querySelector('body')

    body.classList.remove(string.match(/aligned.*/))

    var addCorrection = (className) => {
        body.classList.add(className)
        okay = false
    }

    switch (moveStatus) {
        case "MOVE_UP":
            addCorrection("aligned-up")
            break
        case "MOVE_DOWN":
            addCorrection("aligned-down")
            break
    }

    switch (strafeStatus) {
        case "MOVE_RIGHT":
            addCorrection("aligned-right")
            break
        case "MOVE_LEFT":
            addCorrection("aligned-left")
            break
    }

    var clockwiseArrow = document.querySelector("#clockwise")
    var counterClockwiseArrow = document.querySelector("#counter-clockwise")
    clockwiseArrow.style.display = "none"
    counterClockwiseArrow.style.display = "none"
    switch (turnStatus) {
        case "TURN_CLOCK":
            clockwiseArrow.style.display = "block"
            okay = false
            break
        case "TURN_COUNTERCLOCK":
            counterClockwiseArrow.style.display = "block"
            okay = false
            break
    }
    if (okay) {
        body.classList.add("aligned-complete")
    }
})

NetwordTables.addKeyListener('/SmartDashboard/Single Substation'), (key, poseString) => {
    single_substation_coords = parsePose(poseString)
}

NetworkTables.addKeyListener('/SmartDashboard/Estimated Pose'), (key, poseString) => {
    var field_coord = parsePose(poseString)

    var relX = field_coord[0] - single_substation_coords[0]
    var relY = field_coord[1] - single_substation_coords[1]
    var rotation = field_coord[2]
    
    var robot_box = document.querySelector("#robot")
    robot_box.style.backgroundImage="url(red_triangle.png)"
    robot_box.style.tranform = `translate(${relX}, ${relY})`
    robot_box.style.tranform = `rotate(${rotation}deg)`
}

var parsePose = (poseString) => {
    var x = value.substring(1, value.indexOf(","))
    var y = value.substring(value.indexOf(",")+2 , value.indexOf(")"))
    var rot = value.substring(value.indexOf(")")+2, value.indexOf("degrees")) 
    return [x, y, rot]
}


NetworkTables.addKeyListener('/SmartDashboard/pitch', (key, value) => {
    var line = document.querySelector("#leveling-line")
    var pitchValue = document.querySelector("h3#pitch-value")
    pitchValue.innerHTML = "Pitch: " + Number(value).toFixed(2)
    angle = 2*value
    line.style.transform = `rotate(${angle}deg)`
    if (value == null){
        line.style.backgroundColor = "#ff00ff" //pink
        line.style.borderColor = "#ff00ff" //pink
    }else if (value > 2){
        line.style.backgroundColor = "#FF0000" //red
        line.style.borderColor = "#FF0000" //red
    }else if (value < -2){
        line.style.backgroundColor = "#0000FF" //blue
        line.style.borderColor = "#0000FF" //blue
    }else if (value <= 2 && value >= -2){
        line.style.backgroundColor = "#00FF00" //green
        line.style.borderColor = "#00FF00" //green
    }
})


// NetworkTables.addKeyListener('/SmartDashboard/level', (key, value) => { // TODO change key listener
//     var body = document.querySelector('body')
//     var line = document.querySelector('#leveling-line')
//     if (value) {
//         line.style.transform = "rotate(0deg)"
//         line.style.borderColor = "rgb(8, 218, 8)"
//         if (!body.classList.contains("leveled")) {
//             body.classList.add("leveled")
//         }
//     } else {
//         line.style.transform = `rotate(${angle}deg)`
//         line.style.borderColor = "#ff00ff"
//         if (body.classList.contains("leveled")) {
//             body.classList.remove("leveled")
//         }
//     }
// })

NetworkTables.addKeyListener('/SmartDashboard/target-arm-state', (key, value) => {
    var armState = document.querySelector('div#arm-state h1#arm-state-text')
    armState.innerHTML = ""
    armState.append(document.createTextNode(value))
})

// // updates vision frame
// NetworkTables.addKeyListener('/SmartDashboard/vision-frame-updated', (key, value) => {
//     if (value == true) {
//         document.getElementById('vision-frame').src = document.getElementById('vision-frame').src
//         NetworkTables.putValue('vision-frame-updated', false)
//     }
// })


// // updates robot angle and direction
//     NetworkTables.addKeyListener('/SmartDashboard/vision-values-', (key, value) => {
//         document.getElementById('distance').textContent = 'Distance: ' + value[0]
//         document.getElementById('angle').textContent = 'Angle: ' + value[2]
//     })

// updates status lights for driveBase
NetworkTables.addKeyListener('/SmartDashboard/DriveBase', (key, value) => {
    var subsystem = document.getElementById('drivebase')
    if (value === 'GREEN') {
        subsystem.style.backgroundColor = "rgb(0,255,0)"
    } else if (value === 'YELLOW') {
        subsystem.style.backgroundColor = "rgb(255,255,0)"
    } else if (value === 'RED') {
        subsystem.style.backgroundColor = "rgb(255,0,0)"
    }
})

NetworkTables.addKeyListener('/SmartDashboard/Claw', (key, value) => {
    var subsystem = document.getElementById('claw')
    if (value === 'GREEN') {
        subsystem.style.backgroundColor = "rgb(0,255,0)"
    } else if (value === 'YELLOW') {
        subsystem.style.backgroundColor = "rgb(255,255,0)"
    } else if (value === 'RED') {
        subsystem.style.backgroundColor = "rgb(255,0,0)"
    }
})


NetworkTables.addKeyListener('/SmartDashboard/Vision', (key, value) => {
    var subsystem = document.getElementById('vision')
    if (value === 'GREEN') {
        subsystem.style.backgroundColor = "rgb(0,255,0)"
    } else if (value === 'YELLOW') {
        subsystem.style.backgroundColor = "rgb(255,255,0)"
    } else if (value === 'RED') {
        subsystem.style.backgroundColor = "rgb(255,0,0)"
    }
})


NetworkTables.addKeyListener('/SmartDashboard/Elbow', (key, value) => {
    var subsystem = document.getElementById('elbow')
    if (value === 'GREEN') {
        subsystem.style.backgroundColor = "rgb(0,255,0)"
    } else if (value === 'YELLOW') {
        subsystem.style.backgroundColor = "rgb(255,255,0)"
    } else if (value === 'RED') {
        subsystem.style.backgroundColor = "rgb(255,0,0)"
    }
})


NetworkTables.addKeyListener('/SmartDashboard/Shoulder', (key, value) => {
    var subsystem = document.getElementById('shoulder')
    if (value === 'GREEN') {
        subsystem.style.backgroundColor = "rgb(0,255,0)"
    } else if (value === 'YELLOW') {
        subsystem.style.backgroundColor = "rgb(255,255,0)"
    } else if (value === 'RED') {
        subsystem.style.backgroundColor = "rgb(255,0,0)"
    }
})

NetworkTables.addKeyListener('/SmartDashboard/Wrist', (key, value) => {
    var subsystem = document.getElementById('wrist')
    if (value === 'GREEN') {
        subsystem.style.backgroundColor = "rgb(0,255,0)"
    } else if (value === 'YELLOW') {
        subsystem.style.backgroundColor = "rgb(255,255,0)"
    } else if (value === 'RED') {
        subsystem.style.backgroundColor = "rgb(255,0,0)"
    }
})


NetworkTables.addKeyListener('/SmartDashboard/auton-chooser', (key, value) => {
    var currentPath = document.querySelector("#auto-form h3#current-path")
    var pathText = "Not Sent"
    var listenedAuto = Number(value)
    switch(listenedAuto) { // Todo
        case 0: // ConeCube(driveBase, claw, arm, "CableScore")
            pathText = "cableScore"
            break
        case 1: // ConeCube(driveBase, claw, arm, "StationScore")
            pathText = "stationScore"
            break
        case 2: // CubeEngage(driveBase, claw, arm, "CableEngage")
            pathText = "cableEngage"
            break
        case 3: // CubeEngage(driveBase, claw, arm, "StationEngage")
            pathText = "stationEngage"
            break
        case 4: // CenterEngage(driveBase, claw, arm, "CenterEngage")
            pathText = "centerEngage"
            break
        case 5: // CenterEngage(driveBase, claw, arm, "CenterEngage")
            pathText = "centerIntake"
            break
        case 6: 
            pathText = "scoreMid"
        default:
            pathText = "Not Sent"
    }
    currentPath.innerHTML = ""
    currentPath.append("Current Path: " + pathText)
})




document.getElementById("confirm-button").onclick = () => {
  sendAuton()
}

// // listens for keystrokes from the external keypad and passes the corresponding values over networktables
// var keys = []
// var allKeys = ''
// document.addEventListener("keyup", function(event) {
//     var pressed = event.key.replace("Enter", "")
//     allKeys += pressed
//     var result = allKeys[allKeys.length - 1]
//     var nextTask = getFromMap(result)

//     console.log(nextTask)
//     allKeysPressed.push(nextTask)

//     // make sure the key pressed is a valid action
//     if (nextTask != null) {
//         if (nextTask.toUpperCase() === nextTask) NetworkTables.putValue('/SmartDashboard/xkeys-robotstates', nextTask)
//         else if (nextTask.includes("cancel")) NetworkTables.putValue('/SmartDashboard/xkeys-cancel', nextTask)
//         else if (nextTask.includes("shoot")) NetworkTables.putValue('/SmartDashboard/xkeys-shooter', nextTask)
//         else if (nextTask.includes("updraw")) NetworkTables.putValue('/SmartDashboard/xkeys-updraw', nextTask)
//         else if (nextTask.includes("climber")) NetworkTables.putValue('/SmartDashboard/xkeys-climber', nextTask)
//         else if (nextTask.includes("vision")) NetworkTables.putValue('/Vision/vision-data', nextTask)
//         else if (nextTask.includes("intake") || nextTask.includes("roller")) NetworkTables.putValue('/SmartDashboard/xkeys-intake', nextTask)

//     }
// })

// function getFromMap(key) {

//     // public static final double RUN_INTAKE_IN = 0
//     // public static final double RUN_INTAKE_OUT = 1
//     // public static final double TOGGLE_INTAKE = 2

//     // public static final double INIT_SHOOTER = 4
//     // public static final double SHOOT = 6
//     // public static final double SHOOT_ALL = 7

//     // public static final double INCREASE_SHOOTER_RPM = 8
//     // public static final double DECREASE_SHOOTER_RPM = 9

//     // public static final double INDEXER_INTAKE = 10

//     // public static final double EXTEND_CLIMBER = 12
//     // public static final double RETRACT_CLIMBER = 13

//     // public static final double SHOOT_NEAR = 14
//     // public static final double SHOOT_MID = 15
//     // public static final double SHOOT_LONG = 16

//     // public static final double CANCEL_ALL = 18

//     if (key === "3") return "4"
//     if (key === "5") return "6"
//     if (key === "2") return "7"

//     if (key === "c") return "14"
//     if (key === "j") return "15"
//     if (key === "d") return "16"

//     if (key === "k") return "2"
//     if (key === "e") return "0"
//     if (key === "f") return "1"

//     if (key === "a") return "NEUTRAL"

//     if (key === "m") return "12"
//     if (key === "n") return "13"

//     if (key === "y") return "vision"

//     if (key === "v") return "18"

//     return null
// }


function getAutonFromMap() {
    console.log("SELECTED VALUE", selectedPath)
    switch(selectedPath) { 
        case "cableScore": // ConeCube(driveBase, claw, arm, "CableScore")
            return 0.0
        case "stationScore": // ConeCube(driveBase, claw, arm, "StationScore")
            return 1.0
        case "cableEngage": // CubeEngage(driveBase, claw, arm, "CableEngage")
            return 2.0
        case "stationEngage": // CubeEngage(driveBase, claw, arm, "StationEngage")
            return 3.0
        case "centerEngage": // CenterEngage(driveBase, claw, arm, "CenterEngage")
            return 4.0
        case "centerIntake": // CenterEngage(driveBase, claw, arm, "CenterEngage")
            return 5.0
        case "scoreMid":
            return 6.0
        default:
            return -1
    }
}

function getDelayTime() {
    return parseFloat(document.querySelector('#delay input[name="delay-time"]').value)
}


function sendAuton() {
    var autonCommand = getAutonFromMap()
    var autoSelectWarning = document.querySelector("div#auto-warning") 
    if (autonCommand === -1) {
        autoSelectWarning.style.display = "block"
        return
    } else {
        autoSelectWarning.style.display = "none"
    }
    var delayTime = getDelayTime()
    console.log("SELECTED AUTON COMMAND", autonCommand)
    NetworkTables.putValue('/SmartDashboard/auton-chooser', autonCommand)
    NetworkTables.putValue('/SmartDashboard/delayTime', delayTime)
}