#include "ofMain.h"
#include "NiTE.h"

#define MAX_DEPTH 10000

#define GL_WIN_SIZE_X	1280
#define GL_WIN_SIZE_Y	1024
#define TEXTURE_SIZE	512

#define MIN_NUM_CHUNKS(data_size, chunk_size)	((((data_size)-1) / (chunk_size) + 1))
#define MIN_CHUNKS_SIZE(data_size, chunk_size)	(MIN_NUM_CHUNKS(data_size, chunk_size) * (chunk_size))

class ofxOpenNI2 : public ofThread {
public:
	ofxOpenNI2() : ofThread() {
		niInitilized = false;
	}

	~ofxOpenNI2() {
		if(m_depthStream.isValid()) {
			m_depthStream.destroy();
		}

		delete n_UserTracker;
		nite::NiTE::shutdown();
		openni::OpenNI::shutdown();
	}


	void setup() {
		niInitilized = false;
		n_UserTracker = new nite::UserTracker();


		openni::Status rc =  openni::OpenNI::initialize();
	
		if(rc != openni::STATUS_OK) {
			ofLog() << "Error Initializing Openni: " << openni::OpenNI::getExtendedError();
			return;
		}

		const char* deviceUri = openni::ANY_DEVICE;
		rc = m_device.open(deviceUri);

		if(rc != openni::STATUS_OK) {
			ofLog() << "Can not initialize device: " << openni::OpenNI::getExtendedError();
			return;
		}

		nite::Status nc =  nite::NiTE::initialize();
		if(nc != nite::STATUS_OK) {
			ofLog() << "Coult not initilized NiTE!";
			return;
		}

		nc = n_UserTracker->create(&m_device);
		if(nc != nite::STATUS_OK) {
			ofLog() << "Can not initialize Nite User Tracker";
			return;
		}	

		if(!initDepthStream() )
		{
			return;
		}

		m_videoStreams = new openni::VideoStream*[1];
		m_videoStreams[0] = &m_depthStream;

		niInitilized = true;
	}

	bool initDepthStream() {
		openni::Status rc;

		rc = m_depthStream.create(m_device, openni::SENSOR_DEPTH);
		if(rc == openni::STATUS_OK) {
			rc = m_depthStream.start();

			if(rc != openni::STATUS_OK) {
				ofLog() << "Depth Stream Can Not Start!";
				m_depthStream.destroy();
				return 0;
			}
		} else {
			return 0;
		}
		
		m_depthVideoMode = m_depthStream.getVideoMode();

		m_depth_width = m_depthVideoMode.getResolutionX();
		m_depth_height = m_depthVideoMode.getResolutionY();

		depthPixels[0].allocate(m_depth_width, m_depth_height, OF_IMAGE_COLOR_ALPHA);
		depthPixels[1].allocate(m_depth_width, m_depth_height, OF_IMAGE_COLOR_ALPHA);
		backDepthPixels = &depthPixels[0];
		currentDepthPixels = &depthPixels[1];

		depthImage.allocate(m_depth_width, m_depth_height, OF_IMAGE_COLOR_ALPHA);
		

	}

	void updateDepthStream() {
			updateDepthFrame = false;

			if(m_depthStream.isValid()) {

				int changedIndex;
				openni::Status rc = openni::OpenNI::waitForAnyStream(m_videoStreams, 1, &changedIndex);

				if(rc != openni::STATUS_OK) {
					ofLog() << "Wait failed!";
					unlock();
					return;
				}

				if(changedIndex == 0) {
					m_depthStream.readFrame(&m_depthFrame);
					updateDepthFrame = true;
				}

			}

			if(updateDepthFrame) {

				calculateHistogram(m_pDepthHist, MAX_DEPTH, m_depthFrame);
				
				lock();
				const openni::DepthPixel* depthRow = (const openni::DepthPixel*)m_depthFrame.getData();
				for(int y = 0; y < m_depth_height; y++){
					for(int x = 0; x < m_depth_width; x++, depthRow++){
						backDepthPixels->setColor(x, y, *depthRow);
					}
				}

				const openni::DepthPixel* depth = (const openni::DepthPixel*)m_depthFrame.getData();
				for (int y = m_depthFrame.getCropOriginY(); y < m_depthFrame.getHeight() + m_depthFrame.getCropOriginY(); y++){
					unsigned char * texture = backDepthPixels->getPixels() + y * m_depthFrame.getWidth() * 4 + m_depthFrame.getCropOriginX() * 4;
					for (int x = 0; x < backDepthPixels->getWidth(); x++, depth++, texture += 4) {
						int nHistValue = m_pDepthHist[*depth];
						if (*depth != 0){
							texture[0] = nHistValue;
							texture[1] = nHistValue;
							texture[2] = nHistValue;
						}
					}
				}	

				swap(backDepthPixels, currentDepthPixels);

				bNewDepthPixels = true;
				unlock();

			}
	}

	void threadedFunction() {
		while(isThreadRunning()) {
			// we update the depthPixels
			updateDepthStream();
			sleep(20);
		}
	}

	void updateOnMainThread() {
		lock();
		if(bNewDepthPixels) {
			depthImage.setFromPixels(*currentDepthPixels);
			bNewDepthPixels = false;
		}
		unlock();
		
	}

	void drawDepth() {
		depthImage.draw(0,0);
	}

	void start() {
		if(!niInitilized) return;
		startThread(true, false);
	}

	void stop() {
		stopThread();	
	}

	void calculateHistogram(float* pHistogram, int histogramSize, const openni::VideoFrameRef& frame);
public:
	bool niInitilized;

	openni::Device		m_device;
	nite::UserTracker *n_UserTracker;
	openni::VideoFrameRef m_depthFrame;
	openni::VideoStream m_depthStream;
	
	openni::VideoStream** m_videoStreams;
	openni::VideoMode m_depthVideoMode;
	bool updateDepthFrame;

	float	m_pDepthHist[MAX_DEPTH];

	int			m_depth_width;
	int			m_depth_height;

	bool bNewDepthPixels;
	ofPixels depthPixels[2];
	ofPixels* backDepthPixels;
	ofPixels* currentDepthPixels;
	ofImage depthImage;
};