{
  "startPoint": {
    "x": 18,
    "y": 119,
    "heading": "linear",
    "startDeg": 90,
    "endDeg": 180,
    "locked": false
  },
  "lines": [
    {
      "id": "line-wof3dym847s",
      "name": "shoot close",
      "endPoint": {
        "x": 50,
        "y": 85,
        "heading": "linear",
        "startDeg": 144,
        "endDeg": 180
      },
      "controlPoints": [],
      "color": "#89A7B9",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mm4xdts5-alvrwh",
      "name": "intake close",
      "endPoint": {
        "x": 17.5,
        "y": 83.4,
        "heading": "constant",
        "reverse": false,
        "degrees": 180
      },
      "controlPoints": [],
      "color": "#AD96C6",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mm4xei2d-yioz4x",
      "name": "shoot close",
      "endPoint": {
        "x": 50,
        "y": 85,
        "heading": "constant",
        "reverse": false,
        "degrees": 180
      },
      "controlPoints": [],
      "color": "#75D9BD",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mm4xex1g-em4zet",
      "name": "intake mid",
      "endPoint": {
        "x": 16,
        "y": 59,
        "heading": "constant",
        "reverse": false,
        "degrees": 180
      },
      "controlPoints": [
        {
          "x": 60,
          "y": 56
        }
      ],
      "color": "#5D6AB7",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mm4xfmbj-u3992h",
      "name": "shoot close",
      "endPoint": {
        "x": 50,
        "y": 85,
        "heading": "constant",
        "reverse": false,
        "degrees": 180
      },
      "controlPoints": [],
      "color": "#9DD7A9",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mm4xfy3r-w77qun",
      "name": "intake far",
      "endPoint": {
        "x": 15,
        "y": 35,
        "heading": "constant",
        "reverse": false,
        "degrees": 180
      },
      "controlPoints": [
        {
          "x": 67,
          "y": 30.5
        }
      ],
      "color": "#5C9D57",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mm4xgzfb-lfi2av",
      "name": "shoot close",
      "endPoint": {
        "x": 50,
        "y": 85,
        "heading": "constant",
        "reverse": false,
        "degrees": 180
      },
      "controlPoints": [],
      "color": "#CDC7D5",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mm4xh5lz-afb6dl",
      "name": "park",
      "endPoint": {
        "x": 26,
        "y": 69.6,
        "heading": "constant",
        "reverse": false,
        "degrees": -90
      },
      "controlPoints": [],
      "color": "#6976B5",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    }
  ],
  "shapes": [
    {
      "id": "triangle-1",
      "name": "Red Goal",
      "vertices": [
        {
          "x": 144,
          "y": 70
        },
        {
          "x": 144,
          "y": 144
        },
        {
          "x": 120,
          "y": 144
        },
        {
          "x": 138,
          "y": 119
        },
        {
          "x": 138,
          "y": 70
        }
      ],
      "color": "#dc2626",
      "fillColor": "#ff6b6b"
    },
    {
      "id": "triangle-2",
      "name": "Blue Goal",
      "vertices": [
        {
          "x": 6,
          "y": 119
        },
        {
          "x": 25,
          "y": 144
        },
        {
          "x": 0,
          "y": 144
        },
        {
          "x": 0,
          "y": 70
        },
        {
          "x": 7,
          "y": 70
        }
      ],
      "color": "#2563eb",
      "fillColor": "#60a5fa"
    }
  ],
  "sequence": [
    {
      "kind": "path",
      "lineId": "line-wof3dym847s"
    },
    {
      "kind": "path",
      "lineId": "mm4xdts5-alvrwh"
    },
    {
      "kind": "path",
      "lineId": "mm4xei2d-yioz4x"
    },
    {
      "kind": "path",
      "lineId": "mm4xex1g-em4zet"
    },
    {
      "kind": "path",
      "lineId": "mm4xfmbj-u3992h"
    },
    {
      "kind": "path",
      "lineId": "mm4xfy3r-w77qun"
    },
    {
      "kind": "path",
      "lineId": "mm4xgzfb-lfi2av"
    },
    {
      "kind": "path",
      "lineId": "mm4xh5lz-afb6dl"
    }
  ],
  "settings": {
    "xVelocity": 75,
    "yVelocity": 65,
    "aVelocity": 3.141592653589793,
    "kFriction": 0.1,
    "rWidth": 16,
    "rHeight": 16,
    "safetyMargin": 1,
    "maxVelocity": 40,
    "maxAcceleration": 30,
    "maxDeceleration": 30,
    "fieldMap": "decode.webp",
    "robotImage": "/robot.png",
    "theme": "auto",
    "showGhostPaths": false,
    "showOnionLayers": false,
    "onionLayerSpacing": 3,
    "onionColor": "#dc2626",
    "onionNextPointOnly": false
  },
  "version": "1.2.1",
  "timestamp": "2026-02-27T13:30:14.919Z"
}