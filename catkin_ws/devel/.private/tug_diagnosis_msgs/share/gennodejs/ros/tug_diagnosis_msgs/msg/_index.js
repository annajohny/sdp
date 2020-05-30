
"use strict";

let diagnosis = require('./diagnosis.js');
let configuration = require('./configuration.js');
let observer_configuration = require('./observer_configuration.js');
let diagnosis_set = require('./diagnosis_set.js');
let node_configuration = require('./node_configuration.js');
let resource_mode_assignement = require('./resource_mode_assignement.js');

module.exports = {
  diagnosis: diagnosis,
  configuration: configuration,
  observer_configuration: observer_configuration,
  diagnosis_set: diagnosis_set,
  node_configuration: node_configuration,
  resource_mode_assignement: resource_mode_assignement,
};
