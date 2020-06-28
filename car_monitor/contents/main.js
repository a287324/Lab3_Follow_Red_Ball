var ros = new ROSLIB.Ros({ url : 'ws://' + location.hostname + ':9000' });

ros.on('connection', function() {console.log('websocket: connected'); });
ros.on('error', function(error) {console.log('websocket error: ', error); });
ros.on('close', function() {console.log('websocket: closed');});
/*var ls = new ROSLIB.Topic({
	ros : ros,
	name : '/CamStream/circleNum',
	messageType : 'std_msgs/Int16'
});
	
ls.subscribe(function(message) {
	console.log(message);
});*/

document.getElementById('camstream_original').data = 'http://'
	+ location.hostname
	+ ':10000/stream?topic=/cv_camera/image_raw';

document.getElementById('camstream_result').data = 'http://'
	+ location.hostname
	+ ':10000/stream?topic=/CamStream/image';