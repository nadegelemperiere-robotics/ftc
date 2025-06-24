/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Limelight pipeline management
   ------------------------------------------------------- */
package org.firstinspires.ftc.core.processing.limelight;

/* System includes */
import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.net.HttpURLConnection;
import java.net.URL;
import java.nio.charset.StandardCharsets;

/* JSON includes */
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;

/* Configuration includes */
import org.firstinspires.ftc.core.configuration.Configurable;

public abstract class LimelightPipeline implements Configurable {

    /**
     * Send pipeline configuration
     * @param pipeline the pipeline configuration
     * @param url the limelight rest api url
     * @param index the index of the pipeline to which inputs shall be sent
     * @param logger the logger to use to report status
     */
    boolean                             uploadPipeline(JSONObject pipeline, String url, int index, LogManager logger)
    {

        boolean result = false;

        try {
            URL path = new URL(url + "/upload-pipeline?index=" + index);
            HttpURLConnection conn = (HttpURLConnection) path.openConnection();
            conn.setRequestMethod("POST");
            conn.setDoOutput(true);
            conn.setRequestProperty("Content-Type", "application/json");

            OutputStream out = conn.getOutputStream();
            out.write(pipeline.toString().getBytes(StandardCharsets.UTF_8));
            out.flush();
            out.close();

            if (conn.getResponseCode() != 200) {
                throw new IOException("Pipeline upload failed: " + conn.getResponseCode());
            }
            else { result = true; }

            try (BufferedReader in = new BufferedReader(new InputStreamReader(conn.getInputStream()))) {
                String inputLine = in.readLine();
                while(inputLine != null) {
                    inputLine = in.readLine();
                }
            }
        }
        catch (IOException e) {
            logger.warning("Could not update pipeline : " + e);
        }

        return result;
    }

    /**
     * Bypass updatePythonInputs, which is limited to 32 data by attacking
     * directly the limelight rest API which does not suffer such a limitation
     * @param inputs the input data for python pipeline
     * @param url the limelight rest api url
     * @param index the index of the pipeline to which inputs shall be sent
     * @param logger the logger to use to report status
     */
    boolean                             sendPythonInputs(double[] inputs, String url, int index, LogManager logger)
    {

        boolean result = false;

        try {
            URL path = new URL(url + "/update-pythoninputs");
            HttpURLConnection conn = (HttpURLConnection) path.openConnection();
            conn.setRequestMethod("POST");
            conn.setDoOutput(true);
            conn.setRequestProperty("Content-Type", "application/json");

            JSONArray jsonArray = new JSONArray(inputs);

            OutputStream out = conn.getOutputStream();
            out.write(jsonArray.toString().getBytes(StandardCharsets.UTF_8));
            out.flush();
            out.close();

            if (conn.getResponseCode() != 200) {
                throw new IOException("Failed to send Python input: " + conn.getResponseCode());
            } else { result = true; }

            try (BufferedReader in = new BufferedReader(new InputStreamReader(conn.getInputStream()))) {
                String inputLine = in.readLine();
                while(inputLine != null) {
                    inputLine = in.readLine();
                }
            }
        }
        catch (IOException | JSONException e) {
            logger.warning("Could not send python pipeline data : " + e);
        }

        return result;

    }

    /**
     * Bypass getPythonOutput, which is limited to 32 data by attacking
     * directly the limelight rest API which does not suffer such a limitation
     * @param url the limelight rest api url
     * @param index the index of the pipeline to which inputs shall be sent
     * @param logger the logger to use to report status
     * @return The output data of the pipeline
     */
    double[]                            getPythonOutput(String url, int index, LogManager logger)
    {

        double[] result = null;

        try {
            URL path = new URL(url + "/results");
            HttpURLConnection conn = (HttpURLConnection) path.openConnection();
            conn.setRequestMethod("GET");

            InputStream in = conn.getInputStream();
            InputStreamReader reader = new InputStreamReader(in, StandardCharsets.UTF_8);
            BufferedReader br = new BufferedReader(reader);

            StringBuilder response = new StringBuilder();
            String line;
            while ((line = br.readLine()) != null) {
                response.append(line);
            }

            JSONObject json = new JSONObject(response.toString());
            JSONArray pythonOut = json.getJSONArray("PythonOut");

            result = new double[pythonOut.length()];
            for (int i = 0; i < pythonOut.length(); i++) {
                result[i] = pythonOut.getDouble(i);
            }
        }
        catch (IOException | JSONException e) {
            logger.warning("Could not send python pipeline data : " + e);
        }

        return result;
    }


    /**
     * Send neural network detector tflite model
     * @param model the model file content as byte[]
     * @param url the limelight rest api url
     * @param index the index of the pipeline to which inputs shall be sent
     * @param logger the logger to use to report status
     */
    boolean                             uploadDetectorModel(byte[] model, String url, int index, LogManager logger) {

        boolean result = false;
        try {
            URL path = new URL(url + "/upload-nn?type=detector&index=" + index);
            HttpURLConnection conn = (HttpURLConnection) path.openConnection();
            conn.setRequestMethod("POST");
            conn.setDoOutput(true);
            conn.setRequestProperty("Content-Type", "application/octet-stream");
            conn.setFixedLengthStreamingMode(model.length);

            OutputStream out = conn.getOutputStream();
            out.write(model);
            out.flush();
            out.close();

            if (conn.getResponseCode() != 200) {
                throw new IOException("Upload failed: " + conn.getResponseCode());
            }
            else { result = true; }

            try (BufferedReader in = new BufferedReader(new InputStreamReader(conn.getInputStream()))) {
                String inputLine = in.readLine();
                while(inputLine != null) {
                    inputLine = in.readLine();
                }
            }
        }
        catch (IOException e) {
            logger.warning("Could not send tflite model : " + e);
        }

        return result;
    }

    /**
     * Send neural network detector labels file
     * @param labels the detector labels list
     * @param url the limelight rest api url
     * @param index the index of the pipeline to which inputs shall be sent
     * @param logger the logger to use to report status
     */
    boolean                             uploadDetectorLabels(String labels, String url, int index, LogManager logger) {

        boolean result = false;
        try {

            byte[] message = labels.getBytes(StandardCharsets.UTF_8);

            URL path = new URL(url + "/upload-nnlabels?type=detector&index=" + index);
            HttpURLConnection conn = (HttpURLConnection) path.openConnection();
            conn.setRequestMethod("POST");
            conn.setDoOutput(true);
            conn.setRequestProperty("Content-Type", "application/octet-stream");
            conn.setFixedLengthStreamingMode(message.length);

            OutputStream out = conn.getOutputStream();
            out.write(message);
            out.flush();
            out.close();

            if (conn.getResponseCode() != 200) {
                throw new IOException("Upload failed: " + conn.getResponseCode());
            }
            else { result = true; }

            try (BufferedReader in = new BufferedReader(new InputStreamReader(conn.getInputStream()))) {
                String inputLine = in.readLine();
                while(inputLine != null) { inputLine = in.readLine(); }
            }
        }
        catch (IOException e) {
            logger.warning("Could not send labels : " + e);
        }

        return result;
    }
}
