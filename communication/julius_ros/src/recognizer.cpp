
#include "ros/ros.h"
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <algorithm>
#include <sstream>
#include <boost/thread/thread.hpp>
#include <boost/thread/condition_variable.hpp>

#include "std_msgs/String.h"
#include "std_srvs/Empty.h"

#include <julius/juliuslib.h>

using namespace std;

bool JULIUS_DEBUG = FALSE;
string JULIUS_CONFIGURATION;
ros::Publisher commands;
boost::mutex request_mutex;

boost::condition_variable pause_variable;
boost::mutex pause_mutex;

uint message_sequence = 0;

Jconf *jconf = NULL;
Recog *recog = NULL;

bool running = FALSE;

static inline bool is_not_alnum(char c) { return !(isalnum(c)); }

bool word_valid(const std::string &str) {
	ROS_INFO("Word: %s", str.c_str());

    return find_if(str.begin(), str.end(), is_not_alnum) == str.end();
}

static void status_recready(Recog *recog, void *dummy) {

}

static void status_recstart(Recog *recog, void *dummy) {

}

static void status_paused_wait(Recog *recog, void *dummy) {

	boost::unique_lock<boost::mutex> lock(pause_mutex);

	pause_variable.wait(lock);

}

static void output_result(Recog *recog, void *dummy) {

	int i, j;
	int len;
	WORD_INFO *winfo;
	WORD_ID *seq;
	int seqnum;
	int n;
	Sentence *s;
	RecogProcess *r;
	HMM_Logical *p;
	SentenceAlign *align;

	/* all recognition results are stored at each recognition process instance */
	for(r = recog->process_list; r; r = r->next) {

		/* skip the process if the process is not alive */
		if (! r->live) continue;

		/* result are in r->result.  See recog.h for details */

		/* check result status */
		if (r->result.status < 0) {      /* no results obtained */
		/* outout message according to the status code */
			switch(r->result.status) {
				case J_RESULT_STATUS_REJECT_POWER:
					ROS_DEBUG("<input rejected by power>");
					break;
				case J_RESULT_STATUS_TERMINATE:
					ROS_DEBUG("<input teminated by request>");
					break;
				case J_RESULT_STATUS_ONLY_SILENCE:
					ROS_DEBUG("<input rejected by decoder (silence input result)>");
					break;
				case J_RESULT_STATUS_REJECT_GMM:
					ROS_DEBUG("<input rejected by GMM>");
					break;
				case J_RESULT_STATUS_REJECT_SHORT:
					ROS_DEBUG("<input rejected by short input>");
					break;
				case J_RESULT_STATUS_FAIL:
					ROS_DEBUG("<search failed>");
					break;
			}
			/* continue to next process instance */
			continue;
		}

		/* output results for all the obtained sentences */
		winfo = r->lm->winfo;

		for(n = 0; n < r->result.sentnum; n++) { /* for all sentences */

			s = &(r->result.sent[n]);
			seq = s->word;
			seqnum = s->word_num;

			/* output word sequence like Julius */

			std::stringstream ss;
			
			for(i = 0 ;i < seqnum; i++) {
				if (!word_valid(string(winfo->woutput[seq[i]]))) continue;
				if (!ss.str().empty()) ss << " ";
				ss << winfo->woutput[seq[i]];
			}

			std::string sentence = ss.str();

			std::transform(sentence.begin(), sentence.end(), sentence.begin(), ::tolower);

			ROS_INFO("Recognized sentence: %s", sentence.c_str());

			std_msgs::String msg;
			msg.data = sentence;

			commands.publish(msg);

			/* LM entry sequence */
			/*printf("wseq%d:", n+1);
			for(i=0;i<seqnum;i++) printf(" %s", winfo->wname[seq[i]]);
			printf("\n");*/

			/* phoneme sequence */
			//printf("phseq%d:", n+1);
			//put_hypo_phoneme(seq, seqnum, winfo);
			//printf("\n");

			/* confidence scores */
			/*printf("cmscore%d:", n+1);
			for (i=0;i<seqnum; i++) printf(" %5.3f", s->confidence[i]);
			printf("\n");*/


			/* AM and LM scores */
			ROS_DEBUG("score%d: %f", n+1, s->score);

			if (r->lmtype == LM_PROB) { /* if this process uses N-gram */
				ROS_DEBUG(" (AM: %f  LM: %f)", s->score_am, s->score_lm);
			}

			printf("\n");
			if (r->lmtype == LM_DFA) { /* if this process uses DFA grammar */
				/* output which grammar the hypothesis belongs to
				when using multiple grammars */
				if (multigram_get_all_num(r->lm) > 1) {
					ROS_DEBUG("grammar%d: %d\n", n+1, s->gram_id);
				}
			}

		}
	}


}

bool start_service_callback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {

	boost::lock_guard<boost::mutex> lock(request_mutex);

	if (recog) j_request_resume(recog);

	pause_variable.notify_all();

	return true;

}

bool stop_service_callback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {

	boost::lock_guard<boost::mutex> lock(request_mutex);

	if (recog) j_request_pause(recog);

	return true;

}

void recognize() {

	if (!JULIUS_DEBUG) jlog_set_output(NULL);

	running = TRUE;

	ROS_INFO("Julius rev. %s", JULIUS_VERSION);

	{

		std::vector<char> tmp(JULIUS_CONFIGURATION.begin(), JULIUS_CONFIGURATION.end());
		tmp.push_back(0);
		jconf = j_config_load_file_new(&tmp[0]);

	}
	if (jconf == NULL) {
		ROS_ERROR("Unable to read config file.\n");
		running = FALSE;
		return;
	}
  
	jconf->input.speech_input = SP_MIC;

	recog = j_create_instance_from_jconf(jconf);
	if (recog == NULL) {
		ROS_ERROR("Julius startup error.");
		running = FALSE;
		return;
	}

	/*********************/
	/* Register callback */
	/*********************/
	/* register result callback functions */
	callback_add(recog, CALLBACK_EVENT_SPEECH_READY, status_recready, NULL);
	callback_add(recog, CALLBACK_EVENT_SPEECH_START, status_recstart, NULL);
	callback_add(recog, CALLBACK_RESULT, output_result, NULL);
  callback_add(recog, CALLBACK_PAUSE_FUNCTION, status_paused_wait, NULL);

	/**************************/
	/* Initialize audio input */
	/**************************/
	/* initialize audio input device */
	/* ad-in thread starts at this time for microphone */
	if (j_adin_init(recog) == FALSE) {   
		ROS_ERROR("Failed to initialize audio input.");
		running = FALSE;
		return;
	}

  if (JULIUS_DEBUG) j_recog_info(recog);

	switch(j_open_stream(recog, NULL)) {
		case 0:	
			break;
		case -1:  
			ROS_ERROR("Error in audio input stream.");
			running = FALSE;
			return;
		case -2:
			ROS_ERROR("Failed to start audio input stream.");
			running = FALSE;
			return;
	}

	if (j_recognize_stream(recog) == -1) 
		ROS_ERROR("Speech recognition error.");

	j_close_stream(recog);

	j_recog_free(recog);

	recog = NULL;

	running = FALSE;

}

void watchdog(const ros::TimerEvent&) {

	if (!running) {
		ROS_INFO("Julius is not running, stopping node.");	
		ros::shutdown();
	}

}

int main(int argc, char **argv) {

	ros::init(argc, argv, "recognizer");
	ros::NodeHandle n("~");

	n.param("configuration", JULIUS_CONFIGURATION, string(""));
	n.param("debug", JULIUS_DEBUG, (bool)FALSE);

	if (JULIUS_CONFIGURATION.empty()) {
		ROS_ERROR("No Julius configuration file provided");
		return -1;
	}
  
	boost::thread julius_thread(recognize);

	commands = n.advertise<std_msgs::String>(std::string("output"), 1000);

	ros::ServiceServer start_service = n.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>(ros::this_node::getName() + std::string("/start"), start_service_callback);

	ros::ServiceServer stop_service = n.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>(ros::this_node::getName() + std::string("/stop"), stop_service_callback);

	n.createTimer(ros::Duration(5), watchdog);

	ros::spin();

	if (recog) j_request_terminate(recog);

	pause_variable.notify_all();

	//julius_thread.join();

	return 0;
}

