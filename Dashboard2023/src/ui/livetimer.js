
var countDownTimer;
var endTime;
var matchPhase = document.getElementById("match-phase");
var timer = document.getElementById("timer");
var timerPrefixString = "Time of Match: ";
var phasePrefixString = "Match Phase: ";
var timeoutFunc = null;
var phaseCache = MatchPhases.NOT_STARTED;
var seconds;
var minutes;
const MatchPhases = Object.freeze ({
    NOT_STARTED: Object.freeze( {text: "NOT STARTED", color:"rgb(50,50,50)"}),
    AUTON: Object.freeze({text: "AUTON", color:"rgb(70,70,200)"}),
    TELEOP: Object.freeze({text:"TELEOP",color:"rgb(0,200,0)"}),
    ENDED: Object.freeze({text:"ENDED",color:"rgb(200,0,0)"})
});

setMatchPhase(MatchPhases.NOT_STARTED);
timer.textContent = timerPrefixString;

NetworkTables.addKeyListener("/SmartDashboard/MatchTime", (key, value) => {
    updateTimer(value);
    var timeString = getTimeString(minutes, seconds);
    timer.textContent = timerPrefixString + timeString;
    if (time < 0 && phaseCache == MatchPhases.TELEOP) {
        setMatchPhase(MatchPhases.ENDED);
        timeoutFunc = setTimeout(() => {
            setMatchPhase(MatchPhases.NOT_STARTED); 
            clearTimeout(timeoutFunc);
            timeoutFunc = null;
        }, 5000);
    }
});

NetworkTables.addKeyListener("/SmartDashboard/IsAuton", (key, value) => {
    if (phaseCache == MatchPhases.ENDED ) return;
    if (value) {
        setMatchPhase(MatchPhases.AUTON);
    } else {
        setMatchPhase(MatchPhases.TELEOP);
    }

});


function getTimeString (minutes, seconds) {
    return (seconds == 60 ? minutes + 1 : minutes) + ':' 
        + (Math.round(seconds) < 10 ? "0" : "") 
            + (seconds == 60 ? "00" : Math.round(seconds));
}

function updateTimer(time) {

    seconds =( time % 60);
    minutes = Math.floor( ( time % (60*60)) / 60);
}

function setMatchPhase(phase) {
    phaseCache = phase;
    
    matchPhase.textContent = phasePrefixString + phase.text;
    matchPhase.style.backgroundColor = phase.color;
    if (phase == MatchPhases.AUTON || phase == MatchPhases.NOT_STARTED || phase == MatchPhases.ENDED) {
        document.getElementById("auton-chooser").style.display = "block";
    } else {
        
        document.getElementById("auton-chooser").style.display = "none";
    }
}

