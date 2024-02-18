import { Immutable, MessageEvent, PanelExtensionContext, Topic } from "@foxglove/studio";
//import { useEffect, useLayoutEffect, useState, useMemo,useRef  } from "react";
import { useEffect, useLayoutEffect, useState, useMemo } from "react";
import ReactDOM from "react-dom";
import ReactJson from "react-json-view";

//import { PointCloud, PackedElementField } from "@foxglove/schemas";
import { PointCloud } from "@foxglove/schemas";
type ImageMessage = MessageEvent<PointCloud>;
//type ImageMessage = MessageEvent<CompressedImage>;

//import { CompressedImage } from "@foxglove/schemas";

type PanelState = {
  topic?: string;
};

//async function drawImageOnCanvas(imgData: PackedElementField[]) {
//  console.log(imgData);
//}

//async function drawImageOnCanvas(imgData: Uint8Array, canvas: HTMLCanvasElement, format: string) {
//  const ctx = canvas.getContext("2d");
//
//  if (ctx == undefined) {
//    return;
//  }
//
//  // Create a bitmap from our raw compressed image data.
//  const blob = new Blob([imgData], { type: `image/${format}` });
//  const bitmap = await createImageBitmap(blob);
//
//  // Adjust for aspect ratio.
//  canvas.width = Math.round((canvas.height * bitmap.width) / bitmap.height);
//
//  // Draw the image.
//  ctx.drawImage(bitmap, 0, 0, canvas.width, canvas.height);
//
//  ctx.resetTransform();
//}

function ExamplePanel({ context }: { context: PanelExtensionContext }): JSX.Element {
  const [topics, setTopics] = useState<undefined | Immutable<Topic[]>>();
  const [messages, setMessages] = useState<undefined | Immutable<MessageEvent[]>>();
  const [messages2, setMessages2] = useState<undefined | Immutable<MessageEvent[]>>();

  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();

  const [message, setMessage] = useState<ImageMessage>();
  const imageTopics = useMemo(
    () => (topics ?? []).filter((topic) => topic.datatype === "sensor_msgs/PointCloud2"),
    //() => (topics ?? []).filter((topic) => topic.datatype === "sensor_msgs/CompressedImage"),
    [topics],
  );

  const [count, setCount] = useState(0);
  //const canvasRef = useRef<HTMLCanvasElement>(null);

  // We use a layout effect to setup render handling for our panel. We also setup some topic subscriptions.
  useLayoutEffect(() => {
    // The render handler is run by the broader studio system during playback when your panel
    // needs to render because the fields it is watching have changed. How you handle rendering depends on your framework.
    // You can only setup one render handler - usually early on in setting up your panel.
    //
    // Without a render handler your panel will never receive updates.
    //
    // The render handler could be invoked as often as 60hz during playback if fields are changing often.
    context.onRender = (renderState, done) => {
      //setCount((count) => count + 1);
      // render functions receive a _done_ callback. You MUST call this callback to indicate your panel has finished rendering.
      // Your panel will not receive another render callback until _done_ is called from a prior render. If your panel is not done
      // rendering before the next render call, studio shows a notification to the user that your panel is delayed.
      //
      // Set the done callback into a state variable to trigger a re-render.
      setRenderDone(() => done);

      // We may have new topics - since we are also watching for messages in the current frame, topics may not have changed
      // It is up to you to determine the correct action when state has not changed.
      setTopics(renderState.topics);

      // currentFrame has messages on subscribed topics since the last render call
      //    if (count > 10) {
      //console.log(count);
      if (renderState.currentFrame) {
        //setCount(0);
        setMessages(renderState.currentFrame);
      }
      // }
      if (renderState.currentFrame && renderState.currentFrame.length > 0) {
        setMessage(renderState.currentFrame[renderState.currentFrame.length - 1] as ImageMessage);
      }
    };

    // After adding a render handler, you must indicate which fields from RenderState will trigger updates.
    // If you do not watch any fields then your panel will never render since the panel context will assume you do not want any updates.

    // tell the panel context that we care about any update to the _topic_ field of RenderState
    context.watch("topics");

    // tell the panel context we want messages for the current frame for topics we've subscribed to
    // This corresponds to the _currentFrame_ field of render state.
    context.watch("currentFrame");

    // subscribe to some topics, you could do this within other effects, based on input fields, etc
    // Once you subscribe to topics, currentFrame will contain message events from those topics (assuming there are messages).
    //context.subscribe([{ topic: "/velodyne_points" }]);
  }, [context]);

  const [state, setState] = useState<PanelState>(() => {
    return context.initialState as PanelState;
  });

  useEffect(() => {
    context.saveState({ topic: state.topic });
    if (state.topic) {
      context.subscribe([state.topic]);
    }
  }, [context, state.topic]);

  useEffect(() => {
    if (state.topic == undefined) {
      setState({ topic: imageTopics[0]?.name });
    }
  }, [state.topic, imageTopics]);

  // invoke the done callback once the render is complete
  useEffect(() => {
    renderDone?.();
  }, [renderDone]);

  useEffect(() => {
    if (message) {
      //drawImageOnCanvas(message.message.fields).catch((error) => console.log(error));
    }
  }, [message]);

  //useEffect(() => {
  useLayoutEffect(() => {
    //const interval = setInterval(() => setCount((count) => count + 1), 1000);
    if (messages) {
      setCount((count) => count + 1);
      if (count > 10) {
        //console.log(count);
        //console.log(messages);
        //console.log(typeof messages);
        //const obj = JSON.parse(messages);

        const asyncTest = async () => {
          const convert_json = JSON.stringify(messages);
          //console.log(convert_json);
          const obj = JSON.parse(convert_json);
          console.log(obj[0].message.data);
          setMessages2(messages);
          //await new Promise((resolve) => setTimeout(resolve, 1000));
          //console.log("finish");
          setCount(0);
        };
        asyncTest();
      }
    }
    //return () => clearInterval(interval);
  }, [messages]);

  return (
    <div style={{ height: "100%", padding: "1rem" }}>
      <div style={{ paddingBottom: "1rem", display: "flex", gap: "0.5rem", alignItems: "center" }}>
        <label>Choose a topic to render:</label>

        <select
          value={state.topic}
          onChange={(event) => {
            setState({ topic: event.target.value });
          }}
          style={{ flex: 1 }}
        >
          {imageTopics.map((topic) => (
            <option key={topic.name} value={topic.name}>
              {topic.name}
            </option>
          ))}
        </select>
      </div>
      <h1>カウント: {count}</h1>

      <div>
        <ReactJson src={messages2 ?? {}} />
      </div>
    </div>
    //    <div style={{ padding: "1rem" }}>
    //      <h2>Welcome to your new extension panel!</h2>
    //      <p>
    //        Check the{" "}
    //        <a href="https://foxglove.dev/docs/studio/extensions/getting-started">documentation</a> for
    //        more details on building extension panels for Foxglove Studio.
    //      </p>
    //      <div style={{ display: "grid", gridTemplateColumns: "1fr 1fr", rowGap: "0.2rem" }}>
    //        <b style={{ borderBottom: "1px solid" }}>Topic</b>
    //        <b style={{ borderBottom: "1px solid" }}>Datatype</b>
    //        {(topics ?? []).map((topic) => (
    //          <>
    //            <div key={topic.name}>{topic.name}</div>
    //            <div key={topic.datatype}>{topic.datatype}</div>
    //          </>
    //        ))}
    //      </div>
    //      <div>{messages?.length}</div>
    //    </div>
  );
}

export function initExamplePanel(context: PanelExtensionContext): () => void {
  ReactDOM.render(<ExamplePanel context={context} />, context.panelElement);

  // Return a function to run when the panel is removed
  return () => {
    ReactDOM.unmountComponentAtNode(context.panelElement);
  };
}
