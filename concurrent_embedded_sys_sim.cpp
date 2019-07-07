//comment: Valters Jēkabs Zakrevskis, 99164111, and 28/11/2018.

//pre-processor directives:
#include <iostream>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <string>
#include <vector>
#include <chrono>
#include <random>
#include <map>

using namespace std;

//global constants:
int const MAX_NUM_OF_THREADS = 10;
int const NUM_OF_SAMPLES = 50;
int const NUM_OF_LINKS = 1;

//global variables:
unsigned int count_t = 0, count_p = 0, count_c = 0;
std::mutex mu;   //instance of mutual exclusion lock
std::condition_variable thread_count_cond;
std::unique_lock<std::mutex> locker;
std::atomic<int> active_thread_count(0);

//function prototypes: (as required)
void printCounters(void);

class Sensor {
    /*
     *   abstract base class that models a sensor
     */
    public:
        Sensor(const std::string & type)
        : sensorType(type) {}
        Sensor(std::string & type)
        : sensorType(type) {}

        //get sensor value virtual function
        virtual double getValue() = 0;

        std::string getType() {
            /*returns the type of Sensor that this is:*/
            return sensorType;
        }

        double getRandDouble(double min, double max){
            unsigned int seed = std::chrono::\
                                system_clock::now()\
                                .time_since_epoch().count();
            std::mt19937 generator(seed);
            double k = (double)((unsigned int)generator())\
                                /(unsigned int)generator.max();
            return (max-min)*k + min;
        }
    private:
        string sensorType;
}; //end abstract class Sensor

class TempSensor : public Sensor {
    /*
     *  Class used to get temperature sensor values.
     */
    public:
        TempSensor (const std::string& s = "temperature sensor")
        : Sensor(s) {}
        TempSensor (std::string & s)
        : Sensor(s) {}

        virtual double getValue() {
            /*
             *  return a random value of ambient temperature
             *  between 10 and 30.
             */
            return getRandDouble(10, 30);
        } //end getValue
}; //end class TempSensor

class PressureSensor : public Sensor{
    /*
     *  Class used to get pressure sensor values.
     */
    public:
        PressureSensor (const std::string& s = "pressure sensor")
        : Sensor(s) {}
        PressureSensor (std::string & s)
        : Sensor(s) {}

        virtual double getValue() {
            /*
             *  return a random value of pressure
             *  between 95 and 105.
             */
            return getRandDouble(95, 105);
        } //end getValue
}; //end class PressureSensor

class CapacitiveSensor : public Sensor {
    /*
     *  Class used to get capacitance sensor values.
     */
    public:
        CapacitiveSensor(const std::string& s="capacitive sensor")
        : Sensor(s) {}
        CapacitiveSensor(std::string & s)
        : Sensor(s) {}

        virtual double getValue() {
            /*
             *  return a random value of capacitance
             *  between 1 and 5.
             */
            return getRandDouble(1, 5);
        } //end getValue
}; //end class CapacitiveSensor

class BC {
    /*
     *  Bus controller allows access to sensors one
     *  thread at a time.
    */
    public:
        BC(
            std::vector<Sensor*> & sensors,
            std::mutex & cout_mu,
            std::map<std::thread::id, int>& threadIDs
           ): theSensors(sensors), cout_mu(cout_mu),
              threadIDs(threadIDs), locked(false){}

        void requestBC() {
            /*
             *  Request access to the bus controller. Suspend if
             *  not available, otherwise give access and lock the
             *  BC.
             */
            std::unique_lock<std::mutex> locker(BC_mu);

            //suspend if/while locked and notify cout
            if(locked){
                std::unique_lock<std::mutex> cout_lock(cout_mu);
                cout << " BusController is locked, thread "
                     << threadIDs[std::this_thread::get_id()]
                     << " is about to suspend..\n";
            }
            while(locked == true){  // Suspend while locked
                cond_BC.wait(locker);
            }
            if(locked == false){    // Allow THx access and lock
                locked = true;
                accessID = std::this_thread::get_id();

                // Notify cout about locking by THx
                std::unique_lock<std::mutex> cout_lock(cout_mu);
                cout << " BusController locked by thread "
                          << threadIDs[accessID] << endl;
            }
        }

        double getSensorValue(int selector) {
            /*
             *  Get sensor value corresponding to the selector
             *  sensor type.
             */
            std::unique_lock<std::mutex> locker(BC_mu);

            // If locked by this thread
            if(locked == true &&
               accessID == std::this_thread::get_id()){
                return (*theSensors[selector]).getValue();
            }else{
                std::unique_lock<std::mutex> cout_lock(cout_mu);
                cout << "ERROR 1: accessID != thread ID "
                        "@ getSensorValue()" << endl;
                return -1000.0;
            }
        }

        string getSensorType(int selector){
            /*
             *  Get sensor type corresponding to the selector.
             *  @param:  selector [0,1,2]->[temp, pres, cap]sens
             *  @return: sensor type string
             */
            std::unique_lock<std::mutex> locker(BC_mu);

            // If locked by this thread
            if(locked == true &&
               accessID == std::this_thread::get_id()){
                return (*theSensors[selector]).getType();
            }else{
                std::unique_lock<std::mutex> cout_lock(cout_mu);
                cout << "ERROR 2: accessID != thread ID "
                        "@ getSensorType()" << endl;
                return "ACCESS DENIED(error 2)";
            }
        }

        void releaseBC() {
            /*
             *  Release the bus controller(BC) of called by
             *  the same thread that locked the BC.
             */
            std::unique_lock<std::mutex> locker(BC_mu);

            // If locked by this thread
            if(accessID == std::this_thread::get_id()){
                locked = false;         // Unlock the BC
                std::unique_lock<std::mutex> cout_lock(cout_mu);
                cout << "BusController unlocked by thread "
                     << threadIDs[std::this_thread::get_id()]
                     << endl;
                // Notify the BC suspended threads about free BC
                cond_BC.notify_all();
            }
        }
    private:
        bool locked;                // Bus controller locked?
        std::thread::id accessID;   // ID of allowed access thread
        std::vector<Sensor*>& theSensors;// For sensor interaction
        std::mutex& cout_mu;        // For cout << ... locking
        std::mutex BC_mu;           // For obj specific locking
        std::condition_variable cond_BC;// For notifying free BC
        // Used to output threadID 2 as 0, 3 as 1, 4 as 2, etc.
        std::map<std::thread::id, int>& threadIDs;
}; //end class BC

class SensorData {
    /*
     *  Utility class to store sensor data.
     */
    public:
        SensorData(string type)
        : sensor_type(type){}

        string getSensorType(){
            /* Return sensor type*/
            return sensor_type;
        }
        std::vector<double> getSensorData(){
            /*Return sensor data vector*/
            return sensor_data;
        }
        double getSensorData(int index){
            /* Return a single value from sensor_data vector*/
            return sensor_data[index];
        }
        void addData(double newData){
            /* Add a sensor data point to sensor data vector*/
            sensor_data.push_back(newData);
        }
        void clearData(){
            /* Clear all sensor data vector values*/
            sensor_data.clear();
        }

    private:
        std::string sensor_type;            // Stores sensor type
        std::vector<double> sensor_data;    // Stores sensor data
}; //end class SensorData


class Receiver {
    /*
     *  Object used to receive and store sensor data.
     *  It is accessed via a Link instance.
     */
    public:
        Receiver(std::mutex& cout_mu)  //constructor
        : cout_mu(cout_mu)
        {}

        //Receives a SensorData object:
        void receiveData(SensorData sd) {
            /*
             *  Receive and store the incoming SensorData content.
             */
            std::unique_lock<std::mutex> locker(rx_mu);

            std::string sensor_type = sd.getSensorType();
            std::vector<double> vect = sd.getSensorData();

            //Save data in vectors
            saveData(std::ref(vect), sensor_type);
            {
                std::unique_lock<std::mutex> cout_lock(cout_mu);
                cout << "Data received \t from: "
                     << sensor_type << endl;
            }
        }

        // print out all data for each sensor:
        void printSensorData(int rows = 5){
            /*
             *  Prints out all sensor data in @param rows.
             */
            std::unique_lock<std::mutex> lock(rx_mu);
            {
                std::unique_lock<std::mutex> cout_lock(cout_mu);

                cout << "\nTEMPERATURE DATA" << endl;
                printVectorN(std::ref(temp_data), rows);
                cout << "\nPRESSURE DATA" << endl;
                printVectorN(std::ref(pres_data), rows);
                cout << "\nCAPACITANCE DATA" << endl;
                printVectorN(std::ref(cap_data), rows);
            }
        }

    private:
        void saveData(
                      std::vector<double>& vect,
                      std::string& sensor_type
                      )
        {
            /* Save sensor data.(if many sensor types use a map)*/
            if(sensor_type == "temperature sensor"){
                temp_data.insert(temp_data.end(),
                                 vect.begin(),
                                 vect.end());
            }else if(sensor_type == "pressure sensor"){
                pres_data.insert(pres_data.end(),
                                 vect.begin(),
                                 vect.end());
            }else if(sensor_type == "capacitive sensor"){
                cap_data.insert(cap_data.end(),
                                vect.begin(),
                                vect.end());
            }else{
                std::unique_lock<std::mutex> cout_lock(cout_mu);
                std::cout << "ERROR 3: unknown sensor type @ "
                             "receiveData();" << std::endl;
            }
        }

        void printVectorN(std::vector<double>& vect,
                          int n,
                          std::string seperator = "\t\t")
        {
            /*
             *  Print @param vect in @param n rows seperated by
             *  @param seperator.
             */
            int j = 1;
            for(auto i = vect.begin(); i != vect.end(); ++i){
                cout << *i << seperator;
                if(j++%n == 0)
                    cout<<"\n";
            }
            //Ensure equal new line spacing
            if(--j%n != 0){
                cout << endl;
            }
        }

        std::mutex rx_mu;               // For locking Receiver
        std::mutex& cout_mu;            // For locking cout << ...
        std::vector<double> temp_data;  // Stores all temp data
        std::vector<double> pres_data;  // Stores all pres data
        std::vector<double> cap_data;   // Stores all cap data
}; //end class Receiver

class Link {
    /*
     *  Representation of LinkAccessController communications
     *  link used to transmit data the receiver.
     */
    public:
        Link (Receiver& r, int linkNum) //Constructor
        : inUse(false), myReceiver(r), linkId(linkNum)
        {}

        bool isInUse(){
            /* Check if the link is currently in use*/
            return inUse;
        }
        void setInUse() {
            /* Set the link status to busy*/
            inUse = true;
        }
        void setIdle(){
            /* Set the link status to idle*/
            inUse = false;
        }
        void writeToDataLink(SensorData sd) {
            /* Write data to the receiver*/
            myReceiver.receiveData(sd);
        }
        int getLinkId() {
            /* Returns the link Id*/
            return linkId;
        }
    private:
        bool inUse;             // Is the link being used/locked?
        Receiver& myReceiver;   // Receiving object reference
        int linkId;             // Link unique identifier
}; //end class Link

class LinkAccessController{
    /*
     *  Controls access to all available links, ensuring that
     *  they can be used simultaneously. However, if all
     *  the links are in use and additional threads attempt
     *  to gain access to a link, these latter threads are
     *  suspended until one or more links become available.
     */
    public:
        LinkAccessController(
                        Receiver& r,
                        std::map<std::thread::id, int>& threadIDs,
                        std::mutex& cout_mu
                        )
        : myReceiver(r), numOfAvailableLinks(NUM_OF_LINKS),
          threadIDs(threadIDs), cout_mu(cout_mu)
        {
            for (int i = 0; i < NUM_OF_LINKS; i++) {
                    commsLinks.push_back(Link(myReceiver, i));
            }
        }

        Link& requestLink(){
            /*
             * Request a comm's link: returns a reference to
             * an available Link. If none are available,
             * the calling thread is suspended.
             */
            std::unique_lock<std::mutex> locker(LAC_mu);

            if(numOfAvailableLinks == 0){
                std::unique_lock<std::mutex> cout_lock(cout_mu);
                cout << "<><><><><><>Thread "
                     << threadIDs[std::this_thread::get_id()]
                     << " is about to suspend ... @ "
                        "lac::requestLink()" << endl;
            }
            while(numOfAvailableLinks == 0){
                cond_LAC.wait(locker);  //suspend
            }

            // Iterate through the vector of links until
            // one available link is found. Acquire access
            // to this link.
            int linkNum;
            for(int i = 0; i < NUM_OF_LINKS; i++){
                if (commsLinks[i].isInUse() == false){
                    linkNum = commsLinks[i].getLinkId();
                    commsLinks[i].setInUse();
                    numOfAvailableLinks--;
                    break;
                }
            }

            {
                std::unique_lock<std::mutex> cout_lock(cout_mu);
                cout << "<<<<<<<<<<Thread "
                     << threadIDs[std::this_thread::get_id()]
                     << " has gained access to commsLinks["
                     << linkNum <<"] @ lac::requestLink()"
                     << endl;
            }

            return std::ref(commsLinks[linkNum]);
        }
        void releaseLink(Link& releasedLink) {
            /*
             *  Release a comms link.
             */
            std::unique_lock<std::mutex> locker(LAC_mu);

            releasedLink.setIdle();
            numOfAvailableLinks++;
            cond_LAC.notify_all();

            {
                // print message to cout
                std::unique_lock<std::mutex> cout_lock(cout_mu);
                cout << ">>>>>>>>>>Thread "
                     << threadIDs[std::this_thread::get_id()]
                     << " has released commsLinks["
                     << releasedLink.getLinkId()
                     <<"] @ lac::releaseLink()" << endl;
            }
        }
    private:
        Receiver& myReceiver;           // Receiver reference
        int numOfAvailableLinks;        //
        std::vector<Link> commsLinks;   // Vector of all links
        std::mutex LAC_mu;              // LAC internal mutex
        std::mutex& cout_mu;            // cout << ... print mutex
        std::condition_variable cond_LAC;   // link free cond_var
        std::map<std::thread::id, int>& threadIDs; // Map ID 2 int
}; //end class LinkAccessController



//run function –executed by each thread:
void run(BC & theBC,
         int idx,
         std::mutex& cout_mu,
         std::map<std::thread::id, int>& threadIDs,
         Receiver& receiver,
         LinkAccessController& lac)
{
    /*
     *
     *
     *
     */
    {
        // Add this threads ID to threadIDs map
        std::unique_lock<std::mutex> map_locker(mu);
        threadIDs.insert
            (
                std::make_pair(std::this_thread::get_id(), idx)
            );
    }

    // Create sensor data storage objects (if many sensors are
    // later added modify this to use a vector of SensorData
    // objects)
    SensorData temperatureData("temperature sensor");
    SensorData pressureData("pressure sensor");
    SensorData capacitanceData("capacitive sensor");

    for(int i=0; i< NUM_OF_SAMPLES; i++){
        // 1) request use of the BC:
        theBC.requestBC();

        // 2) generates a random integer (selector)
        //    between 0 and 2.
        //      0 -> temperature sensor
        //      1 -> pressure sensor
        //      2 -> capacitive sensor
        //    Get the selected sensor's type and value
        unsigned int seed = std::chrono::system_clock::now()\
                            .time_since_epoch().count();
        std::mt19937 generator(seed);
        int selector = generator()%3;

        //Get sensor value and type
        double sensor_value = theBC.getSensorValue(selector);
        string sensor_type = theBC.getSensorType(selector);

        //Save data in its storage object
        if(sensor_type == "temperature sensor"){
            temperatureData.addData(sensor_value);
        }else if(sensor_type == "pressure sensor"){
            pressureData.addData(sensor_value);
        }else if(sensor_type == "capacitive sensor"){
            capacitanceData.addData(sensor_value);
        }else{
            std::unique_lock<std::mutex> cout_lock(cout_mu);
            std::cout << "\t\t\t\tError 4: unknown sensor type"
                      << std::endl;
        }

        // 3) Print out the sensor value obtained with ID
        {
            std::unique_lock<std::mutex> cout_lock(cout_mu);
            cout << "\tsample value from thread "
                 << threadIDs[std::this_thread::get_id()]
                 << " from " << sensor_type
                 << " = " << sensor_value << endl;
        }
        // 4) increment counter for sensor chosen
        // (to keep count of how many times each was used)
        if (sensor_type == "temperature sensor"){
            count_t++;
        }else if (sensor_type == "pressure sensor"){
            count_p++;
        }else if (sensor_type == "capacitive sensor"){
            count_c++;
        }else{
            std::unique_lock<std::mutex> cout_lock(cout_mu);
            cout << "\t\t\t\tERROR 5: tried to increment"
                    "an unknown sensor type" << endl;
        }
        // 5) release the BC:
        theBC.releaseBC();

        // 6) delay for random period between 0.001s – 0.01s:
        int ms = generator()%10 + 1;
        std::this_thread::sleep_for(
                                    std::chrono::milliseconds(ms)
                                    );
         /*
        Link* link  = &lac.requestLink();
            link->writeToDataLink(temperatureData);
                temperatureData.clearData();
            link->writeToDataLink(pressureData);
                pressureData.clearData();
            link->writeToDataLink(capacitanceData);
                capacitanceData.clearData();
        lac.releaseLink(*link);

        //Delay 1 to 10 ms
        seed = std::chrono::system_clock::now().time_since_epoch()\
               .count();
        generator.seed(seed);
        ms = generator()%10 + 1;
        std::this_thread::sleep_for (std::chrono::milliseconds(ms));
         */
    }
    active_thread_count--;
    thread_count_cond.notify_all();

    // /*
    Link* link  = &lac.requestLink();
        link->writeToDataLink(temperatureData);
            temperatureData.clearData();
        link->writeToDataLink(pressureData);
            pressureData.clearData();
        link->writeToDataLink(capacitanceData);
            capacitanceData.clearData();
    lac.releaseLink(*link);

    //Delay 1 to 10 ms
    unsigned int seed = std::chrono::system_clock::now()\
                        .time_since_epoch().count();
    std::mt19937 generator(seed);
    int ms = generator()%10 + 1;
    std::this_thread::sleep_for (std::chrono::milliseconds(ms));
    // */
}

int main() {
    // Declare a vector of Sensor pointers:
    std::vector<Sensor*> sensors;
    // Initialise each sensor and insert into the vector:
    sensors.push_back(new TempSensor()); //push_back is a vector method.
    sensors.push_back(new PressureSensor());
    sensors.push_back(new CapacitiveSensor());

    // Mutex used for cout printing
    std::mutex cout_mu;
    // map used to print out thread IDs starting from zero (2,3,4)->(0,1,2)
    std::map<std::thread::id, int> threadIDs;

    // Instantiate the BC:
    BC theBC(std::ref(sensors),
             std::ref(cout_mu),
             std::ref(threadIDs));

    // Instantiate the Receiver:
    Receiver receiver(std::ref(cout_mu));

    // Instantiate the LinkAccessController
    LinkAccessController lac(std::ref(receiver),
                             std::ref(threadIDs),
                             std::ref(cout_mu));

    //instantiate and start the threads:
    std::thread the_threads[MAX_NUM_OF_THREADS];
    for (int i = 0; i < MAX_NUM_OF_THREADS; i++) {
        //launch the threads:
        the_threads[i] = std::thread(run,
                                     std::ref(theBC),
                                     i,
                                     std::ref(cout_mu),
                                     std::ref(threadIDs),
                                     std::ref(receiver),
                                     std::ref(lac));
        active_thread_count++;
    }

    //wait for the threads to finish:
    for (int i = 0; i < MAX_NUM_OF_THREADS; i++)
        the_threads[i].join();

    //wait for the threads to finish:
    while(active_thread_count != 0){
        std::unique_lock<std::mutex> cout_lock(cout_mu);
        thread_count_cond.wait(cout_lock);
        std::cout << "active_thread_count = "
                  << active_thread_count << std::endl;
    }

    // Print out the number of times each sensor was accessed:
    printCounters();

    // Print all sensor data
    receiver.printSensorData();

    cout << endl << "All threads terminated" << endl << endl;

    return 0;
}

void printCounters(void){
    //print out the number of times each sensor was accessed:
    cout << "_______________________________" << endl;
    cout << "Sensor\t\t\tCount" << endl;
    cout << "_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _" << endl;
    cout << "Temperature\t\t"<< count_t << endl;
    cout << "Pressure\t\t"<< count_p << endl;
    cout << "Capacitative\t\t"<< count_c << endl;
    cout << "_______________________________" << endl;
    cout << "total count\t\t" << count_t + count_p + count_c << endl;
    return;
}
