let address = document.getElementById('connect-address'),
  connect = document.getElementById('connect')

let loginShown = true;

// Set function to be called on NetworkTables connect. Not implemented.
//NetworkTables.addWsConnectionListener(onNetworkTablesConnection, true);

// Set function to be called when robot dis/connects
NetworkTables.addRobotConnectionListener(onRobotConnection, false);

// Sets function to be called when any NetworkTables key/value changes
//NetworkTables.addGlobalListener(onValueChanged, true);

// Function for hiding the connect box
onkeydown = key => {
  if (key.key === 'Escape') {
    document.body.classList.toggle('login', false);
    loginShown = false;
  }
};

/**
 * Function to be called when robot connects
 * @param {boolean} connected
 */
function onRobotConnection(connected) {
  var state = connected ? 'Robot connected!' : 'Robot disconnected.';
  console.log(state);
  // ui.robotState.textContent = state;
  if (loginShown){
    setLogin();
    var d2 = new Date();
    console.log('>>>time: ' + d2.getSeconds() + '.' + d2.getMilliseconds());
  }
  if (connected) {
    // On connect hide the connect popup
    document.body.classList.toggle('login', false);
    document.getElementById('login').style.display = "none";
    loginShown = false;
  } else {
    console.log("Disconnected");
    ipc.send('connect', address.value);
  }
}
function setLogin() { 
  // Add Enter key handler
  // Enable the input and the button
  address.disabled = connect.disabled = false;
  connect.textContent = 'Connect';
  // Add the default address and select xxxx
  address.value = '10.6.70.2';
  address.focus();
  address.setSelectionRange(8, 12);
  ipc.send('connect', address.value);
}

// On click try to connect and disable the input and the button
// connect.onclick = () => {
//   ipc.send('connect', address.value, 80);
//   address.disabled = connect.disabled = true;
//   connect.textContent = 'Connecting...';
// };

address.onkeydown = ev => {
  if (ev.key === 'Enter') {
    connect.click();
    ev.preventDefault();
    ev.stopPropagation();
  }
};

// Show login when starting
document.body.classList.toggle('login', true);
setLogin();
