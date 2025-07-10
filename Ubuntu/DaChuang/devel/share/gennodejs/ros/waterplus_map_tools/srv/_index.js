
"use strict";

let GetChargerByName = require('./GetChargerByName.js')
let AddNewWaypoint = require('./AddNewWaypoint.js')
let GetNumOfWaypoints = require('./GetNumOfWaypoints.js')
let GetWaypointByName = require('./GetWaypointByName.js')
let SaveWaypoints = require('./SaveWaypoints.js')
let GetWaypointByIndex = require('./GetWaypointByIndex.js')

module.exports = {
  GetChargerByName: GetChargerByName,
  AddNewWaypoint: AddNewWaypoint,
  GetNumOfWaypoints: GetNumOfWaypoints,
  GetWaypointByName: GetWaypointByName,
  SaveWaypoints: SaveWaypoints,
  GetWaypointByIndex: GetWaypointByIndex,
};
