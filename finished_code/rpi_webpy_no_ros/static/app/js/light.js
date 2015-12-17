(function() {
    'use strict';

/* Controllers */

	var controllers = angular.module('light', []);

	controllers.controller('LightCtrl', ['$scope', '$http', '$interval', '$location', lightCtrl]);

  function lightCtrl($scope, $http, $interval, $location) {
  
    var baseUrl = "http://192.168.1.14:8080"
    // var baseUrl = "http://10.251.85.58:8080"

    $scope.colors = [];

    $scope.go = function(s) {
      $location.path(s);
    }

    $scope.stopCmd = function (){
      
      $http.post(baseUrl + "/req/q").success(function() {
            console.log('success');
        }).error(function() {
            console.log('error');
        });
    };

    $scope.sendDest = function (){
      var custom = document.getElementById("destInput").value;
      
      $http.post(baseUrl + "/req/" + custom).success(function() {
            console.log('success');
        }).error(function() {
            console.log('error');
        });
    };
    
    $scope.drawLine = function(x1, y1, x2, y2, text) {
      var c = document.getElementById("myCanvas");
      var ctx = c.getContext("2d");

      ctx.fillStyle = 'black';
      ctx.strokeStyle = 'black';
      // ctx.fillText(text.toString(), x1 + 5, y1 + 1);

      ctx.beginPath();
      ctx.moveTo(x1, y1);
      ctx.lineTo(x2, y2);
      ctx.stroke();

      ctx.beginPath();
      ctx.arc(x1, y1, 3, 0, 2 * Math.PI, false);
      ctx.fillStyle = 'black';
      ctx.fill();
      ctx.lineWidth = 1;
      ctx.strokeStyle = '#000000';
      ctx.stroke();

      // ctx.beginPath();
      // ctx.fillStyle = '#00FF00';
      // ctx.strokeStyle = '#FF00FF';
      // ctx.font = "30px Arial";
      // console.log(text.toString());
      // ctx.fillText("TSETSETSETSETSETSETSET", x1 + 5, y1 + 5);
      // ctx.fillText("asdae2w3ds", 0, 0);
      // ctx.strokeText("asdae2w3ds", 0, 0);
      // ctx.fill();

      // ctx.fillText("asdae2w3ds", 0, 0);
      // ctx.strokeText("asdae2w3ds", 0, 0);

      ctx.fillStyle = 'black';
      ctx.strokeStyle = 'black';
    }

    $scope.setCanvasSize = function(s) {
      var c = document.getElementById("myCanvas");
      c.width = s;
      c.height = s;
    }

    $scope.updateBotPos = function(data, i) {

      if (data == undefined) { return; }

      var c = document.getElementById("myCanvas");
      var ctx = c.getContext("2d");
      ctx.clearRect(0, 0, c.width, c.height);
      $scope.drawMapLines();

      ctx.stroke();

      ctx.fillStyle="red";
      var x = data.x;
      var y = data.y;

      if (i == 1) {
        x = (data.x2 + data.x) / 2;
        y = (data.y2 + data.y) / 2;
      } else if (i == 2) {
        x = data.x2;
        y = data.y2;
      }

      ctx.beginPath();
      ctx.arc(x, y, 10, 0, 2 * Math.PI, false);
      ctx.fill();
      ctx.lineWidth = 1;
      ctx.strokeStyle = '#000000';
      ctx.stroke();

      ctx.fillStyle="black";
    }

    $scope.callbackCnt = 0;

    var updateCallback = function() {
        
        console.log("Beat");

        if ($scope.callbackCnt == 0) {

        var httpRequest = $http({
              method: 'GET',
              url: baseUrl + '/pos'
          }).success(function(data, status) {
              $scope.lastPosData = data;
              $scope.updateBotPos(data, 0);
          });
        } else {
          $scope.updateBotPos($scope.lastPosData, $scope.callbackCnt);
        }

        $scope.callbackCnt = ($scope.callbackCnt + 1) % 3;
    };

    updateCallback();
    $interval(updateCallback, 400);

    $scope.drawMapLines = function() {
        var idx = 1;
        if ($scope.mapData != undefined) {
        angular.forEach($scope.mapData.lines, function (item) {
          $scope.drawLine(item.x1, item.y1, item.x2, item.y2, idx);
          idx++;
        });
      }
    };

    $scope.stopInterval = function() {
        if (angular.isDefined(updateTimer)) {
          $interval.cancel(updateTimer);
          updateTimer = undefined;
        }
    };  

    $scope.$on('$destroy', function() {
        // Make sure that the interval is destroyed too
        $scope.stopInterval();
    });
    
    function onLoad(){

        var httpRequest = $http({
              method: 'GET',
              url: baseUrl + '/map'
          }).success(function(data, status) {

              $scope.mapData = data;

              $scope.setCanvasSize(data.size);
              console.log($scope.mapData)

              $scope.drawMapLines();
              
          });
    };
    
    onLoad();
  }
})();  