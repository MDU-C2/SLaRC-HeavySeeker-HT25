import { useEffect, useRef, useState } from "react";
import { createRoot } from "react-dom/client";
import maplibregl from "maplibre-gl";
import "maplibre-gl/dist/maplibre-gl.css";
import type { PanelExtensionContext } from "@foxglove/studio";

type PointStamped = {
  header: { frame_id: string; stamp: { sec: number; nsec: number } };
  point: { x: number; y: number; z: number };
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
  const goalMarkerRef = useRef<maplibregl.Marker | null>(null);
  const robotMarkerRef = useRef<maplibregl.Marker | null>(null);

  const [initialCenter, setInitialCenter] = useState<[number, number] | null>(null);
  const [pubReady, setPubReady] = useState(false);

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

      if (!goalMarkerRef.current) {
        goalMarkerRef.current = new maplibregl.Marker({ color: "#e53935" })
          .setLngLat([lng, lat])
          .addTo(mapRef.current!);
      } else {
        goalMarkerRef.current.setLngLat([lng, lat]);
      }


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

  return (
    <div style={{ width: "100%", height: "100%", position: "relative" }}>
      <div ref={containerRef} style={{ position: "absolute", inset: 0 }} />
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