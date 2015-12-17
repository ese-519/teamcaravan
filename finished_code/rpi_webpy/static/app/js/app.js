'use strict';

/* App Module */

var app = angular.module('app', [
  'ngRoute',
  'animations',

  'light',
  'filters',
  'services'
]);

app.config(['$routeProvider',
  function($routeProvider) {
    $routeProvider.
      when('/light', {
        templateUrl: 'partials/light.html',
      }).
      otherwise({
        redirectTo: '/light'
      });
  }]);
