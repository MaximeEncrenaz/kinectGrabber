#include "testApp.h"


//--------------------------------------------------------------
void testApp::setup() {
	ofSetLogLevel(OF_LOG_VERBOSE);
	
	// enable depth->video image calibration
	kinect.setRegistration(true);
    
	kinect.init();
	//kinect.init(true); // shows infrared instead of RGB video image
	//kinect.init(false, false); // disable video image (faster fps)
	
	kinect.open();		// opens first available kinect
	//kinect.open(1);	// open a kinect by id, starting with 0 (sorted by serial # lexicographically))
	//kinect.open("A00362A08602047A");	// open a kinect using it's unique serial #
	
	// print the intrinsic IR sensor values
	if(kinect.isConnected()) {
		ofLogNotice() << "sensor-emitter dist: " << kinect.getSensorEmitterDistance() << "cm";
		ofLogNotice() << "sensor-camera dist:  " << kinect.getSensorCameraDistance() << "cm";
		ofLogNotice() << "zero plane pixel size: " << kinect.getZeroPlanePixelSize() << "mm";
		ofLogNotice() << "zero plane dist: " << kinect.getZeroPlaneDistance() << "mm";
	}

	
	colorImg.allocate(kinect.width, kinect.height);
	grayImage.allocate(kinect.width, kinect.height);
	grayThreshNear.allocate(kinect.width, kinect.height);
	grayThreshFar.allocate(kinect.width, kinect.height);
	
	nearThreshold = 230;
	farThreshold = 70;
	bThreshWithOpenCV = true;
	
	ofSetFrameRate(60);
	
	// zero the tilt on startup
	angle = 0;
	kinect.setCameraTiltAngle(angle);
    
    // Init
    m_bIsRecording = false;
    
    // Default init
    g_bRecordSignal = false;
    g_bDisplay = true;
	
}

//--------------------------------------------------------------
void testApp::update() {
	
	ofBackground(100, 100, 100);
	
	kinect.update();
	
	// there is a new frame and we are connected
	if(kinect.isFrameNew()) {
		
		// load grayscale depth image from the kinect source
		grayImage.setFromPixels(kinect.getDepthPixels(), kinect.width, kinect.height);
		

        unsigned char * pix = grayImage.getPixels();
		
        
        // Grayscale rescaling between near and far threshold
        int numPixels = grayImage.getWidth() * grayImage.getHeight();
        for(int i = 0; i < numPixels; i++) {
            if(pix[i] > nearThreshold or pix[i] < farThreshold) {
				pix[i] = 0;
			} else {
				pix[i] = 255*(pix[i]-farThreshold)/(nearThreshold-farThreshold);
			}
        }
        
        img.setFromPixels(grayImage.getPixels(), kinect.width, kinect.height, OF_IMAGE_GRAYSCALE, false);
        
		if(m_bIsRecording){
            img.saveImage("images/"+ofToString(m_iFrame)+".jpg"); // change the ".jpg" for ".png" if you want a png sequence.
            m_iFrame++;
        }
		// update the cv images
		grayImage.flagImageChanged();
		
	}
	
}

//--------------------------------------------------------------
void testApp::draw() {
	
	ofSetColor(255, 255, 255);
	

    // draw from the live kinect
    
    kinect.drawDepth(10, 10, 400, 300);
    //kinect.draw(420, 10, 400, 300);
		
    grayImage.draw(10, 320, 400, 300);
	
	// draw instructions
	ofSetColor(255, 255, 255);
	stringstream reportStream;
    
	reportStream << "press r to start recording" << endl
	<< "using opencv threshold = " << bThreshWithOpenCV <<" (press spacebar)" << endl
	<< "set near threshold " << nearThreshold << " (press: + -)" << endl
	<< "set far threshold " << farThreshold << " (press: < >) " 	<< ", fps: " << ofGetFrameRate() << endl
	<< "press c to close the connection and o to open it again, connection is: " << kinect.isConnected() << endl;

    if(kinect.hasCamTiltControl()) {
    	reportStream << "press UP and DOWN to change the tilt angle: " << angle << " degrees" << endl;
    }
    
	ofDrawBitmapString(reportStream.str(), 20, 652);
    
    if (m_bIsRecording )
        record();
}


//--------------------------------------------------------------
void testApp::exit() {
	kinect.setCameraTiltAngle(0); // zero the tilt on exit
	kinect.close();
	
}

int testApp::record() {
    
    setFileNameStringNow(m_sSequenceFileName);
    cout << "[app] Recording sequence to " << m_sSequenceFileName << endl;
    return 0;
}

//--------------------------------------------------------------
void testApp::keyPressed (int key) {
	switch (key) {
        case 'r':
            m_bIsRecording ^=true;
		case ' ':
			bThreshWithOpenCV = !bThreshWithOpenCV;
			break;
			
		case'p':
			bDrawPointCloud = !bDrawPointCloud;
			break;
			
		case '>':
		case '.':
			farThreshold ++;
			if (farThreshold > 255) farThreshold = 255;
			break;
			
		case '<':
		case ',':
			farThreshold --;
			if (farThreshold < 0) farThreshold = 0;
			break;
			
		case '+':
		case '=':
			nearThreshold ++;
			if (nearThreshold > 255) nearThreshold = 255;
			break;
			
		case '-':
			nearThreshold --;
			if (nearThreshold < 0) nearThreshold = 0;
			break;
			
		case 'w':
			kinect.enableDepthNearValueWhite(!kinect.isDepthNearValueWhite());
			break;
			
		case 'o':
			kinect.setCameraTiltAngle(angle); // go back to prev tilt
			kinect.open();
			break;
			
		case 'c':
			kinect.setCameraTiltAngle(0); // zero the tilt
			kinect.close();
			break;
			
			
		case OF_KEY_UP:
			angle++;
			if(angle>30) angle=30;
			kinect.setCameraTiltAngle(angle);
			break;
			
		case OF_KEY_DOWN:
			angle--;
			if(angle<-30) angle=-30;
			kinect.setCameraTiltAngle(angle);
			break;
	}
}

//--------------------------------------------------------------
int testApp::setFileNameStringNow(string& a_sFilename) {
    
    int time_c = clock();
    
	time_t timer;
	time(&timer);
	struct tm * timeinfo;
	timeinfo = localtime ( &timer );
    
    std::ostringstream stringStream;
    stringStream
    << timeinfo->tm_year + 1900 << "-"
    << timeinfo->tm_mon+1 << "-"
    << timeinfo->tm_mday << "T"
    << timeinfo->tm_hour << "_"
    << timeinfo->tm_min << "_"
    << timeinfo->tm_sec << ".oni";
    
    a_sFilename = stringStream.str();
    
    return 0;
}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button)
{}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button)
{}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button)
{}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h)
{}
