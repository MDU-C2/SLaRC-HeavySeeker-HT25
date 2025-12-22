import { useEffect, useRef, useState, ReactNode } from "react";
import { createRoot } from "react-dom/client";
import maplibregl from "maplibre-gl";
import "maplibre-gl/dist/maplibre-gl.css";
import "./ClickToGoalPanel.css";
import { Command } from "./types/Command";
import type { PanelExtensionContext } from "@foxglove/studio";


type PointStamped = {
  header: { frame_id: string; stamp: { sec: number; nsec: number } };
  point: { x: number; y: number; z: number };
};

type Action = {
  id: string;
  label: string;
  icon?: ReactNode;
  onClick: () => void;
};

function nowStamp() {
  const t = Date.now() / 1000;
  const sec = Math.floor(t);
  const nsec = Math.floor((t - sec) * 1e9);
  return { sec, nsec };
}

export function ClickToGoalPanel({ context }: { context: PanelExtensionContext }) {
  const containerRef = useRef<HTMLDivElement | null>(null);
  const mapRef = useRef<maplibregl.Map | null>(null);
  //const goalMarkerRef = useRef<maplibregl.Marker | null>(null);
  const robotMarkerRef = useRef<maplibregl.Marker | null>(null);

  const [initialCenter, setInitialCenter] = useState<[number, number] | null>(null);
  const [pubReady, setPubReady] = useState(false);
  const [isRunning, setIsRunning] = useState(false);

  const waypointMarkersRef = useRef<maplibregl.Marker[]>([]);
  const [errorMessage, setErrorMessage] = useState<string | null>(null);


  useEffect(() => {

    try{
      context.watch?.("currentFrame");
      context.subscribe?.([{ topic: "/gps/filtered" }]);
    }catch(e){
      console.log("Subscribe error: "+e)
    }


    context.onRender = (renderState, done) => {
      void renderState
      if (!pubReady) {
        try {
          context?.advertise?.("/clicked_point", "geometry_msgs/msg/PointStamped");
          setPubReady(true);
        } catch (e) {
          console.log("advertise /clicked_point failed:", e);
        }
      }
      
      // Get the latest /gps/filtered message from currentFrame and move robot-marker
      const frame = renderState.currentFrame;
      if (frame && frame.length > 0) {
        const gpsMessages = frame.filter((m: any) => m.topic === "/gps/filtered");

        if (gpsMessages.length > 0) {
          const lastGps = gpsMessages[gpsMessages.length - 1]; 

          const msg: any = lastGps?.message;
          const lat = msg.latitude;
          const lng = msg.longitude;

          if (typeof lat === "number" && typeof lng === "number") {
            if (!initialCenter) {
              setInitialCenter([lat, lng]);
            }
            if(mapRef.current){
              if (!robotMarkerRef.current) {
                robotMarkerRef.current = new maplibregl.Marker({ color: "#37a5d1ff" })
                  .setLngLat([lng, lat])
                  .addTo(mapRef.current);
              } else {
                robotMarkerRef.current.setLngLat([lng, lat]);
              }
            }
          }
        }
      }
      // Call done when you've rendered all the UI for this renderState.
      // If your UI framework delays rendering, call done when rendering has actually happened.
      done();
    };

    return;
  }, [context, pubReady, initialCenter]);

  useEffect(() => {
    // vi behöver ett container-element, inget befintligt map-objekt, OCH en initialCenter för att starta upp kartan på robotens position
    if (!containerRef.current || mapRef.current || !initialCenter) {
      return;
    }

    const [lat0, lng0] = initialCenter;

    mapRef.current = new maplibregl.Map({
      container: containerRef.current,
      style: {
        version: 8,
        sources: {
          osm: {
            type: "raster",
            tiles: [
              "https://tile.openstreetmap.org/{z}/{x}/{y}.png",
            ],
            tileSize: 256,
            attribution:
              "© OpenStreetMap contributors",
          },
        },
        layers: [
          {
            id: "osm",
            type: "raster",
            source: "osm",
          },
        ],
      },
      center: [lng0, lat0],
      zoom: 13,
      maxZoom: 19,
    });
    mapRef.current.addControl(new maplibregl.NavigationControl());

    mapRef.current.on("click", (e) => {
      const lng = e.lngLat.lng;
      const lat = e.lngLat.lat;

      if (!mapRef.current) return;

      const index = waypointMarkersRef.current.length;
      const marker = createNumberedMarker(index)
        .setLngLat([lng, lat])
        .addTo(mapRef.current);

      waypointMarkersRef.current.push(marker);


      if (pubReady && context.publish) {
        const msg: PointStamped = {
          header: { 
            frame_id: "map",       
            stamp: nowStamp(),
          },
          point: {
            x: lng,                 
            y: lat,           
            z: 0.0,
          },
        };
        context.publish("/clicked_point", msg);
        console.log(msg)
      }
    });
  }, [context, initialCenter]);


  async function sendWaypointCommand(context: any, command: number, index = -1) {
    try {
      const response = await context.callService("waypoint_command", {
        command,
        waypoint_index: index
      });

      console.log("Response:", response);
      return response
    } catch (err) {
      console.error("Service call failed:", err);
      return { success: false, message: err || "Service call failed" };
    }
  }


  function createNumberedMarker(index: number) {
    const el = document.createElement("div");
    el.className = "goal-marker";
    el.innerText = String(index + 1); 

    return new maplibregl.Marker({ element: el });
  }


  const clearAllMarkers = () => {
    waypointMarkersRef.current.forEach(m => m.remove());
    waypointMarkersRef.current = [];
  };

  const removeLastMarker = () => {
    const last = waypointMarkersRef.current.pop();
    last?.remove();
  };


  function showTemporaryError(message: string, duration = 3000) {
    setErrorMessage(message);
    setTimeout(() => setErrorMessage(null), duration);
  }


  const handleStart = async () => {
    console.log("Start clicked");
    const result = await sendWaypointCommand(context, Command.START);
    if (result.success){
        setIsRunning(true);
        setErrorMessage(null);
    }
    else{
        showTemporaryError(result.message || "Failed to start waypoint navigation.");
    }
  };

  const handlePause = async () => {
    console.log("Pause clicked");
    const result = await sendWaypointCommand(context, Command.STOP);
    if (result.success){
        setIsRunning(false);
        setErrorMessage(null);
    }
    else{
        showTemporaryError(result.message || "Failed to pause waypoint navigation.");
    }
  };

  const handleUndo = async () => {
    console.log("Undo clicked");
    const result = await sendWaypointCommand(context, Command.UNDO);
    if (result.success){
      removeLastMarker();
      setErrorMessage(null);
    }
    else{
        showTemporaryError(result.message || "Failed to undo last waypoint.");
    }
  };

  const handleClear = async () => {
    console.log("Clear all clicked");
    const result = await sendWaypointCommand(context, Command.CLEAR_ALL);
    if (result.success){
      clearAllMarkers();
      setErrorMessage(null);
    }
    else{
        showTemporaryError(result.message || "Failed to clear waypoints.");
    }
  };


  const actions: Action[] = [
    {
      id: "startPause",
      label: isRunning ? "Pause" : "Start",
      icon: isRunning ? "⏸️" : "▶️",
      onClick: () => {
        if (isRunning) {
          handlePause();
        } else {
          handleStart();
        }
      },
    },
    {
      id: "clear",
      label: "Clear all",
      icon: "🗑️",
      onClick: () => {
        handleClear();
      },
    },
    {
      id: "undo",
      label: "Undo",
      icon: "↩️",
      onClick: () => {
        handleUndo();
      },
    },
  ];


  return (
    <div style={{ width: "100%", height: "100%", position: "relative" }}>
      <div ref={containerRef} style={{ position: "absolute", inset: 0 }} />
            <div
        style={{
          position: "absolute",
          left: 0,
          right: 0,
          bottom: 0,
          padding: "12px 16px",
          background: "rgba(0, 0, 0, 0.55)",
          display: "flex",
          borderTopRightRadius:12,
          borderTopLeftRadius:12,
          flexWrap: "wrap",
          gap: 8,
          justifyContent: "space-around",
          alignItems: "center",
          zIndex: 10,
        }}
      >
        {actions.map((action) => (
          <button
            key={action.id}
            onClick={action.onClick}
            style={{
              flex: 1,
              minWidth: 100,
              height:40,
              display: "flex",
              alignItems: "center",
              justifyContent: "center",
              gap: 8,
              padding: "10px 12px",
              background: "rgba(255,255,255,0.7)",
              color: "#111",
              border: "none",
              borderRadius: 8,
              cursor: "pointer",
              fontSize: 14,
              fontWeight: 600,
            }}
          >
            {action.icon && <span>{action.icon}</span>}
            <span>{action.label}</span>
          </button>
        ))}
      </div>
      {!initialCenter && (
        <div
          style={{
            position: "absolute",
            left: 8,
            top: 8,
            padding: "6px 8px",
            background: "rgba(0,0,0,0.6)",
            color: "white",
            borderRadius: 8,
            fontSize: 12,
          }}
        >
          Waiting for first GPS fix on <code>/gps/filtered</code>...
        </div>
      )}
      {errorMessage && (
      <div
        style={{
          position: "absolute",
          bottom: 60,
          left: 8,
          padding: "8px 12px",
          background: "rgba(255,0,0,0.8)",
          color: "white",
          borderRadius: 8,
          fontSize: 12,
          zIndex: 20,
        }}
      >
        {errorMessage}
      </div>
    )}
    </div>
  );
}

export function mountPanel(context: PanelExtensionContext) {
  const root = createRoot(context.panelElement);
  root.render(<ClickToGoalPanel context={context} />);
  (context as any).onDestroy?.(() => root.unmount());
}