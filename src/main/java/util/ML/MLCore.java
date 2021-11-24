package util.ML;

import com.google.gson.*;
import frc.robot.Logging;

import java.lang.reflect.Type;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class MLCore {
    private static MLCore mlCore;
    private final Gson mlInputParser;
    public static final double ANGLE_PER_PIXEL_X = 1.125 / 2;
    public static final double ANGLE_PER_PIXEL_Y = 0.75;
    public static final int CAMERA_WIDTH = 640;
    public static final int CAMERA_HEIGHT = 480;

    public static MLCore getInstance() {
        if (mlCore == null) mlCore = new MLCore();
        return mlCore;
    }

    public MLCore() {
        GsonBuilder builder = new GsonBuilder();
        builder.registerTypeAdapter(MLResponse.class, new MLResponseDeserializer());
        mlInputParser = builder.create();
    }

    public Map<String, List<MLObject>> getDetectedObjects() {
        MLResponse resp = getResponse();
        Map<String, List<MLObject>> ret = new HashMap<>();
        for(var obj : resp.objects) {
            if(!ret.containsKey(obj.label)) ret.put(obj.label, new ArrayList<>());
            ret.get(obj.label).add(obj);
        }
        return ret;
    }

    public MLResponse getResponse() {
        try {
            String mlOut = Logging.networkTableInstance.getTable("ML").getEntry("detections").getString("");
            return mlInputParser.fromJson(mlOut, MLResponse.class);
        } catch (Exception e) {
            e.printStackTrace();
        }
        return new MLResponse();
    }

    private class MLResponseDeserializer implements JsonDeserializer<MLResponse> {
        public MLResponse deserialize(JsonElement json, Type typeOfT, JsonDeserializationContext context)
                throws JsonParseException {
            MLResponse response = new MLResponse();
            var detectedObjects = json.getAsJsonArray();

            for (var object : detectedObjects) {
                var jsonObject = object.getAsJsonObject();
                var boundingBoxJson = jsonObject.get("box").getAsJsonObject();
                var boundingBox = new BoundingBox(
                        boundingBoxJson.get("ymin").getAsInt(),
                        boundingBoxJson.get("ymax").getAsInt(),
                        boundingBoxJson.get("xmin").getAsInt(),
                        boundingBoxJson.get("xmax").getAsInt()
                );

                var tmp = new MLObject(jsonObject.get("label").getAsString(), boundingBox, jsonObject.get("confidence").getAsDouble());
                response.objects.add(tmp);
            }

            return response;
        }
    }
}
