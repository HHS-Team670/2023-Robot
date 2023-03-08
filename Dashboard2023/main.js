'use strict';
Object.defineProperty(exports, '__esModule', { value: true });

const electron = require('electron');
const child_process = require("child_process");
var path = require("path");

/** Module to control application life. */
const app = electron.app;

/** Module to create native browser window.*/
const BrowserWindow = electron.BrowserWindow;

// Keep a global reference of the window object, if you don't, the window will
// be closed automatically when the JavaScript object is garbage collected.
/**
 * The Main Window of the Program
 * @type {Electron.BrowserWindow}
 * */

// Keep a global reference of the window object, if you don't, the window will
// be closed automatically when the JavaScript object is garbage collected.
let mainWindow, cameraWindow;

// Define global reference to the python server (which we'll start next).
let server;

function startServer() {
    console.log("starting server")
    let isWin = process.platform === "win32";
    // Start python server.
    if (isWin) {
        // If on Windows, use the batch command (py -3 ./server.py).
        server = child_process.spawn("py", ["-3", "-m", "pynetworktables2js", "--robot=127.0.0.1"]);
        // server = child_process.spawn("py", ["-3", "-m", "pynetworktables2js", "--robot=roborio-670-frc.local"]);
        // server = child_process.spawn("py", ["-3", "-m", "pynetworktables2js", "--robot=10.6.70.2"]);
        // server = child_process.spawn("py", ["-3", "-m", "pynetworktables2js", "--team=670"]);
    }
    else {
        // If on unix-like/other OSes, use bash command (python3 ./server.py).
        server = child_process.spawn("python3", ["-m", "pynetworktables2js", "--robot=127.0.0.1"]);
        // server = child_process.spawn("python3", ["-m", "pynetworktables2js", "--robot=roborio-670-frc.local"]);
        // server = child_process.spawn("python3", ["-m", "pynetworktables2js", "--robot=10.6.70.2"]);
        // server = child_process.spawn("python3", ["-m", "pynetworktables2js", "--team=670"]);
    }
    // On an error close the window and display an error message to the user
    server.on("error", error => {
        console.log("Python Server Error", `Error: ${error.message}. Please check that you have python3 installed and ensure that ${isWin ? "py" : "python3"} is in your PATH.
		https://www.python.org/downloads/`);
        electron.dialog.showErrorBox("Python Server Error", `Error: ${error.message}. Please check that you have python3 installed and ensure that ${isWin ? "py" : "python3"} is in your PATH.
		https://www.python.org/downloads/`);
        mainWindow.close();
    });
    // On server exiting before being killed
    server.on("exit", errCode => {
        // If the servers exits without an error
        if (errCode === 0) {
            console.log("Python Server Exit", "Server Exited");
            electron.dialog.showErrorBox("Python Server Exit", "Server Exited");
            return;
        }
        // Reads the error
        let errorMsg = server.stderr.read().toString().trim();
        // Gets the Module name
        let pythonExtract = /module named (\w+)/.exec(errorMsg);
        let extraMsg = "";
        let moduleInstalled = false;
        // If the server exited due to a module not being installed
        if (pythonExtract !== null && pythonExtract[1]) {
            let moduleName;
            moduleName = pythonExtract[1];
            // If windows then try to install the module
            if (isWin) {
                let { status, stderr } = child_process.spawnSync("py", ["-3", "-m", "pip", "install", moduleName]);
                if (status === 0) {
                    moduleInstalled = true;
                }
                else {
                    // If module installation failed then inform the user why
                    extraMsg = `
                    Running:
                    py -3 -m pip install ${moduleName}
					Error message:
                    ${stderr.toString()}`;
                }
            }
            else {
                // If not on Windows, inform the user how to install the module
                extraMsg = ` Try running 'pip3 install ${moduleName}' (append '--user' if you don't have root access).`;
            }
        }
        if (moduleInstalled) {
            // Restart the server if the problem was resolved
            startServer();
        }
        else {
            // Close the window if the problem was not resolved
            mainWindow.close();
            electron.dialog.showErrorBox("Python Error", errorMsg + extraMsg);
            console.log(errorMsg + extraMsg)
        }
    });
}

// let connectedFunc,
//     ready = false;

// let clientDataListener = (key, val, valType, mesgType, id, flags) => {
//     if (val === 'true' || val === 'false') {
//         val = val === 'true';
//     }
//     mainWindow.webContents.send(mesgType, {
//         key,
//         val,
//         valType,
//         id,
//         flags
//     });
// };


function createWindow() {
    // Create the browser window.
    mainWindow = new BrowserWindow({
        webPreferences: {
        //preload: path.join(app.getAppPath(), 'preload.js')
            nodeIntegration: true,
            contextIsolation: false
        },
        width: 1500,
        height: 900,
        // 1366x570 is a good standard height, but you may want to change this to fit your DriverStation's screen better.
        // It's best if the dashboard takes up as much space as possible without covering the DriverStation application.
        // The window is closed until the python server is ready
        show: false,
        icon: __dirname + '/../images/icon.png'
    });

    cameraWindow = new BrowserWindow({
        webPreferences: {
        //preload: path.join(app.getAppPath(), 'preload.js')
            nodeIntegration: true,
            contextIsolation: false
        },
        width: 1500,
        height: 900,
        // 1366x570 is a good standard height, but you may want to change this to fit your DriverStation's screen better.
        // It's best if the dashboard takes up as much space as possible without covering the DriverStation application.
        // The window is closed until the python server is ready
        show: false,
        icon: __dirname + '/../images/icon.png'
    });

    // Move window to top right of screen.
    mainWindow.setPosition(0, 0);
    cameraWindow.setPosition(0, 0);
    // mainWindow.setPosition(940, 0);
    // Load window.

    mainWindow.loadFile(path.join(__dirname, 'index.html'));
    // mainWindow.loadURL('http://127.0.0.1:8888') //TODO

    cameraWindow.loadURL('http://10.6.70.2:1181') // camera stream URL
    // cameraWindow.loadURL('http://google.com')
    // Once the python server is ready, load window contents.

    mainWindow.once('ready-to-show', () => {
        console.log('main window is ready to be shown');
        mainWindow.webContents.reload();
        mainWindow.show();
    });

    // Remove menu
    mainWindow.setMenu(null);
    // Emitted when the window is closed.
    mainWindow.on('closed', () => {
        console.log('main window closed');
        // Dereference the window object, usually you would store windows
        // in an array if your app supports multi windows, this is the time
        // when you should delete the corresponding element.
        mainWindow = null;
        // ready = false;
        // connectedFunc = null;
        // client.removeListener(clientDataListener);
    });
    mainWindow.on('unresponsive', () => {
        console.log('Main Window is unresponsive');
    });
    mainWindow.webContents.on('did-fail-load', () => {
        console.log('window failed load');
    });
    mainWindow.once('ready-to-show', () => {
        console.log('main window is ready to be shown');
        mainWindow.webContents.reload();
        mainWindow.show();
    });

    // Remove menu
    cameraWindow.setMenu(null);
    // Emitted when the window is closed.
    cameraWindow.on('closed', () => {
        console.log('camera window closed');
        // Dereference the window object, usually you would store windows
        // in an array if your app supports multi windows, this is the time
        // when you should delete the corresponding element.
        cameraWindow = null;
        // ready = false;
        // connectedFunc = null;
        // client.removeListener(clientDataListener);
    });
    cameraWindow.on('unresponsive', () => {
        console.log('Camera Window is unresponsive');
    });
    cameraWindow.webContents.on('did-fail-load', () => {
        console.log('window failed load');
    });
    cameraWindow.once('ready-to-show', () => {
        console.log('main window is ready to be shown');
        cameraWindow.webContents.reload();
        cameraWindow.show();
    });
}
// This method will be called when Electron has finished
// initialization and is ready to create browser windows.
app.on('ready', () => {
    console.log('app is ready');
    startServer();
    createWindow();
});

// Quit when all windows are closed.
app.on('window-all-closed', function () {
    // On OS X it is common for applications and their menu bar
    // to stay active until the user quits explicitly with Cmd + Q.
    // Not like we're creating a consumer application though.
    // Let's just kill it anyway.
    // If you want to restore the standard behavior, uncomment the next line.
    // if (process.platform !== "darwin")
    app.quit();
});

app.on('quit', function () {
    console.log('Application quit.');
    // Kill tornado server child process.
    server.kill("SIGINT");
});

app.on('activate', function () {
    // On OS X it's common to re-create a window in the app when the
    // dock icon is clicked and there are no other windows open.
    if (server == null) {
        startServer();
    }
    if (mainWindow == null) {
        createWindow();
    }
});
