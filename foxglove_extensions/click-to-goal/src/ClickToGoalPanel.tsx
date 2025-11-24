import { useEffect, useRef, useState, ReactNode } from "react";
import { createRoot } from "react-dom/client";
import maplibregl from "maplibre-gl";
import "maplibre-gl/dist/maplibre-gl.css";
import type { PanelExtensionContext } from "@foxglove/studio";
import "./ClickToGoalPanel.css";


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

      // if (!goalMarkerRef.current) {
      //   goalMarkerRef.current = new maplibregl.Marker({ color: "#eb1834ff" })
      //     .setLngLat([lng, lat])
      //     .addTo(mapRef.current!);
      // } else {
      //   goalMarkerRef.current.setLngLat([lng, lat]);
      // }

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


  function createNumberedMarker(index: number) {
    const el = document.createElement("div");
    el.className = "goal-marker";
    el.innerText = String(index + 1); // 1,2,3,...

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


  const handleStart = () => {
    console.log("Start clicked");
    // TODO: start waypoint following here
    setIsRunning(true);
  };

  const handlePause = () => {
    console.log("Pause clicked");
    // TODO: pause/stop waypoint following here
    setIsRunning(false);
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
        console.log("Reset clicked");
        clearAllMarkers();
        setIsRunning(false);
      },
    },
    {
      id: "undo",
      label: "Undo",
      icon: "↩️",
      onClick: () => {
        console.log("Other clicked");
        removeLastMarker();
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
    </div>
  );
}

export function mountPanel(context: PanelExtensionContext) {
  const root = createRoot(context.panelElement);
  root.render(<ClickToGoalPanel context={context} />);
  (context as any).onDestroy?.(() => root.unmount());
}