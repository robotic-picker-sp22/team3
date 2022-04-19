export const rvizConfig = {
    "globalOptions": {
      "background": "#111111",
      "colladaLoader": "collada2",
      "colladaServer": "http://localhost:8001/",
      "fixedFrame": "/map",
      "url": "ws://localhost:9090",
      "videoServer": "http://localhost:9999"
    },
    "sidebarOpened": false,
    "displays": [
      {
        "isShown": true,
        "name": "Grid",
        "options": {
          "cellSize": "1",
          "color": "#cccccc",
          "numCells": "10"
        },
        "type": "grid"
      },
      {
        "isShown": true,
        "name": "Map",
        "options": {
          "color": {
            "r": 255,
            "g": 255,
            "b": 255
          },
          "continuous": true,
          "opacity": "1",
          "topic": "/map"
        },
        "type": "occupancyGrid"
      },
      {
        "isShown": true,
        "name": "Robot model",
        "options": {
          "param": "robot_description"
        },
        "type": "urdf"
      },
      {
        "isShown": true,
        "name": "Interactive Markers",
        "options": {
          "topic": "/map_annotator/map_poses"
        },
        "type": "interactiveMarkers"
      }
    ]
  }