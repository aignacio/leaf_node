
var PORT = 7878;
var HOST = 'aaaa::1';

var dgram = require('dgram');
var server = dgram.createSocket('udp6');

server.on('listening', function () {
    var address = server.address();
    console.log('UDP Server listening on ' + address.address + ":" + address.port);
});

server.on('message', function (message, remote) {
    console.log(remote.address + ' Port:' + remote.port +' - ' + message);
});

server.bind(PORT, HOST);
