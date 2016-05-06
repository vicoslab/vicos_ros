package si.vicos.ros;

import java.util.ArrayList;
import java.util.Locale;

import java.util.concurrent.LinkedBlockingDeque;
import java.util.concurrent.TimeUnit;

import java.io.DataInputStream;
import java.io.PrintWriter;
import java.io.IOException;
import java.net.Socket;
import java.net.UnknownHostException;

import android.app.Activity;
import android.content.ActivityNotFoundException;
import android.content.Intent;
import android.os.Bundle;
import android.os.Handler;
import android.speech.RecognizerIntent;
import android.view.Menu;
import android.view.View;
import android.widget.ImageButton;
import android.widget.TextView;
import android.widget.Toast;
import android.util.Log;

import android.content.SharedPreferences;
import android.preference.PreferenceManager;

public class VoiceCommanderActivity extends Activity implements SharedPreferences.OnSharedPreferenceChangeListener {

	public static final String LOG_TAG = "VoiceCommander";
	private TextView txtSpeechInput;
	private TextView txtClientStatus;
	private ImageButton btnSpeak, btnSettings;
	private final int REQ_CODE_SPEECH_INPUT = 100;
    private final int CONNECTION_WAIT = 1000;

    private ClientConnection client = null;

    private Handler handler = new Handler();

    class ClientConnection implements Runnable {
        private String ip;
        private int port;

        private boolean running = true;

        private Socket socket = null;
        private PrintWriter stream = null;

        private LinkedBlockingDeque<String> queue = new LinkedBlockingDeque<String>();

        private Thread thread;

        public ClientConnection(String ip, int port) {

            this.ip = ip;
            this.port = port;

            thread = new Thread(this);
            thread.start();

        }

        public void run() {

            while (running) {

                try {

                    if (socket == null) {
                        socket = new Socket(this.ip, this.port);
                        stream = new PrintWriter(socket.getOutputStream());
                        Log.d(VoiceCommanderActivity.LOG_TAG, "Connected to: " + this.ip);
                        updateStatus(true);
                    }

                    String msg = queue.poll(CONNECTION_WAIT, TimeUnit.SECONDS);

                    if (msg == null || msg.isEmpty()) continue;

                    Log.d(VoiceCommanderActivity.LOG_TAG, "Sending command: " + msg);

                    stream.println(msg);
                    stream.flush();

                } catch (IOException e) {
                    socket = null;
                    updateStatus(false);
                    try {
                        Thread.sleep(CONNECTION_WAIT);
                    } catch (InterruptedException e1) {}

                } catch (InterruptedException e) {
                    
                }

            }

        }

        private void updateStatus(boolean enabled) {
            if (enabled) {
                handler.post(new Runnable() {
                    public void run() {
                        btnSpeak.setEnabled(true);
                        txtClientStatus.setText(VoiceCommanderActivity.this.getResources().getString(R.string.status_connected));
                    }
                });
            } else {

                handler.post(new Runnable() {
                    public void run() {
                        btnSpeak.setEnabled(false);
                        txtClientStatus.setText(VoiceCommanderActivity.this.getResources().getString(R.string.status_notconnected));
                    }
                });
            }
        }

        public synchronized void stop () {

            if (socket == null)
                return;

            try {
                socket.close();
            } catch (IOException e) {}

            running = false;

            queue.add("");

        }

        public synchronized void send(String msg) {

            Log.d(VoiceCommanderActivity.LOG_TAG, "Commiting command: " + msg);
            queue.add(msg);

        }

    }

	@Override
	protected void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		setContentView(R.layout.activity_main);

        txtClientStatus = (TextView) findViewById(R.id.txtClientStatus); 
		txtSpeechInput = (TextView) findViewById(R.id.txtSpeechInput);
		btnSpeak = (ImageButton) findViewById(R.id.btnSpeak);
		btnSettings = (ImageButton) findViewById(R.id.btnSettings);

		getActionBar().hide();

		btnSpeak.setOnClickListener(new View.OnClickListener() {

			@Override
			public void onClick(View v) {
				promptSpeechInput();
			}
		});

		btnSettings.setOnClickListener(new View.OnClickListener() {

			@Override
			public void onClick(View v) {
				promptSettingsActivity();
			}
		});


	}

	private void promptSpeechInput() {
		Intent intent = new Intent(RecognizerIntent.ACTION_RECOGNIZE_SPEECH);
		intent.putExtra(RecognizerIntent.EXTRA_LANGUAGE_MODEL,
				RecognizerIntent.LANGUAGE_MODEL_FREE_FORM);
		intent.putExtra(RecognizerIntent.EXTRA_LANGUAGE, Locale.getDefault());
		intent.putExtra(RecognizerIntent.EXTRA_PROMPT,
				getString(R.string.speech_prompt));
		try {
			startActivityForResult(intent, REQ_CODE_SPEECH_INPUT);
		} catch (ActivityNotFoundException a) {
			Toast.makeText(getApplicationContext(),
					getString(R.string.speech_not_supported),
					Toast.LENGTH_SHORT).show();
		}
	}

	private void promptSettingsActivity() {
		Intent intent = new Intent(VoiceCommanderActivity.this, SettingsActivity.class);

		try {
			startActivity(intent);
		} catch (ActivityNotFoundException e) {
            Log.e(VoiceCommanderActivity.LOG_TAG, "Failed to open activity: "
							+ e.getMessage());
		}
	}

	/**
	 * Receiving speech input
	 * */
	@Override
	protected void onActivityResult(int requestCode, int resultCode, Intent data) {
		super.onActivityResult(requestCode, resultCode, data);

		switch (requestCode) {
		case REQ_CODE_SPEECH_INPUT: {
			if (resultCode == RESULT_OK && null != data) {

				ArrayList<String> result = data
						.getStringArrayListExtra(RecognizerIntent.EXTRA_RESULTS);

				txtSpeechInput.setText(result.get(0));
                if (client != null) {
                    client.send(result.get(0));
                }

			}
			break;
		}

		}
	}

	@Override
	public void onStop() {

        super.onStop();
        // Log.d(VoiceCommanderActivity.LOG_TAG, "Stopping");
        if (client != null) {
            client.stop();
            client = null;
        }

        PreferenceManager.getDefaultSharedPreferences(this).unregisterOnSharedPreferenceChangeListener(this);
	}

	@Override
	public void onStart() {

        super.onStart();

        SharedPreferences sharedPreferences = PreferenceManager.getDefaultSharedPreferences(this);

        sharedPreferences.registerOnSharedPreferenceChangeListener(this);

        client = new ClientConnection(sharedPreferences.getString("proxy_ip", "127.0.0.1"), 
            Integer.parseInt(sharedPreferences.getString("proxy_port", "5000")));
        //Log.d(VoiceCommanderActivity.LOG_TAG, "Starting");
	}

    public void onSharedPreferenceChanged(SharedPreferences sharedPreferences, String key) {
        if (client != null) {
            client.stop();
            client = null;
        }

        client = new ClientConnection(sharedPreferences.getString("proxy_ip", "127.0.0.1"), 
            Integer.parseInt(sharedPreferences.getString("proxy_port", "5000")));
    }

}
