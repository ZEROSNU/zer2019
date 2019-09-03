// import yaml from 'js-yaml';
// import fs from 'fs';
// src = fs.readFileSync(path.join(__dirname, 'preliminary.yaml'), 'utf8');
// const env = yaml.safeLoad(fs.readFileSync(__dirname + '/preliminary.yaml', 'utf8'));
// console.log(env);
var map;
var latitude = 37.240140; // YOUR LATITUDE VALUE
var longitude = 126.773742; // YOUR LONGITUDE VALUE
var label;

var markers = [];

function save() {
console.log(markers);
}

function getLabel() {
label = document.getElementById("label").value;
}


function initMap() {
//var geoloccontrol = new klokantech.GeolocationControl(map, mapMaxZoom);
var myLatLng = {lat: latitude, lng: longitude};

map = new google.maps.Map(document.getElementById('map'), {
zoom: 17,
center: new google.maps.LatLng(37.240140, 126.773742),
disableDoubleClickZoom: true, // disable the default map zoom on double click
mapTypeId: google.maps.MapTypeId.SATELLITE
});
// Update lat/long value of div when anywhere in the map is clicked
google.maps.event.addListener(map,'click',function(event) {
document.getElementById('latclicked').innerHTML = event.latLng.lat();
document.getElementById('longclicked').innerHTML =  event.latLng.lng();
});

var marker = new google.maps.Marker({
position: myLatLng,
map: map,
// setting latitude & longitude as title of the marker
// title is shown when you hover over the marker
title: latitude + ', ' + longitude
});

// Update lat/long value of div when the marker is clicked
marker.addListener('click', function(event) {
document.getElementById('latclicked').innerHTML = event.latLng.lat();
document.getElementById('longclicked').innerHTML =  event.latLng.lng();
});

// Create new marker on double click event on the map
google.maps.event.addListener(map,'dblclick',function(event) {
getLabel()
lat = event.latLng.lat()
lng = event.latLng.lng()
var marker = new google.maps.Marker({
position: event.latLng,
map: map,
title: lat + ', ' + lng,
// title: event.latLng.lat()+', '+event.latLng.lng(),
label: label,
});
markers.push([label, lat, lng]);

// Update lat/long value of div when the marker is clicked
marker.addListener('click', function() {
document.getElementById('latclicked').innerHTML = event.latLng.lat();
document.getElementById('longclicked').innerHTML =  event.latLng.lng();
});
});

