/*
 * Copyright (C) 2018 dimercur
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "tasks.h"
#include <stdexcept>

// Déclaration des priorités des taches
#define PRIORITY_TSERVER 30
#define PRIORITY_TOPENCOMROBOT 20
#define PRIORITY_TMOVE 20
#define PRIORITY_TSENDTOMON 22
#define PRIORITY_TRECEIVEFROMMON 25
#define PRIORITY_TSTARTROBOT 20
#define PRIORITY_TCAMERA 21
#define PRIORITY_TBATTERYLEVEL 40
#define PRIORITY_TWATCHD 50
#define PRIORITY_TWATCHCOROBOT 45
#define PRIORITY_TSENDTOROBOT 24
#define PRIORITY_TRESETALL 42


/*
 * Some remarks:
 * 1- This program is mostly a template. It shows you how to create tasks, semaphore
 *   message queues, mutex ... and how to use them
 * 
 * 2- semDumber is, as name say, useless. Its goal is only to show you how to use semaphore
 * 
 * 3- Data flow is probably not optimal
 * 
 * 4- Take into account that ComRobot::Write will block your task when serial buffer is full,
 *   time for internal buffer to flush
 * 
 * 5- Same behavior existe for ComMonitor::Write !
 * 
 * 6- When you want to write something in terminal, use cout and terminate with endl and flush
 * 
 * 7- Good luck !
 */

/**
 * @brief Initialisation des structures de l'application (tâches, mutex, 
 * semaphore, etc.)
 */
void Tasks::Init() {
    int status;
    int err;

    /**************************************************************************************/
    /* 	Mutex creation                                                                    */
    /**************************************************************************************/
    if (err = rt_mutex_create(&mutex_monitor, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_robot, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_robotStarted, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_move, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_robotMode, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_err_count_robot, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    
    cout << "Mutexes created successfully" << endl << flush;

    /**************************************************************************************/
    /* 	Semaphors creation       							  */
    /**************************************************************************************/
    if (err = rt_sem_create(&sem_barrier, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_openComRobot, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_serverOk, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_startRobot, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_startWatchD, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_comRobotOK, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_reset, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    
    cout << "Semaphores created successfully" << endl << flush;

    /**************************************************************************************/
    /* Tasks creation                                                                     */
    /**************************************************************************************/
    if (err = rt_task_create(&th_server, "th_server", 0, PRIORITY_TSERVER, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_sendToMon, "th_sendToMon", 0, PRIORITY_TSENDTOMON, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_receiveFromMon, "th_receiveFromMon", 0, PRIORITY_TRECEIVEFROMMON, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_openComRobot, "th_openComRobot", 0, PRIORITY_TOPENCOMROBOT, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_startRobot, "th_startRobot", 0, PRIORITY_TSTARTROBOT, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_move, "th_move", 0, PRIORITY_TMOVE, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_batteryLevel, "th_batterylevel", 0, PRIORITY_TBATTERYLEVEL, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    
    if (err = rt_task_create(&th_WatchD, "th_WatchD", 0, PRIORITY_TWATCHD, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_watchCoRobot, "th_WatchCoRobot", 0, PRIORITY_TWATCHCOROBOT, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_sendToRobot, "th_sendToRobot", 0, PRIORITY_TSENDTOROBOT, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    
    if (err = rt_task_create(&th_ResetAll, "th_ResetAll", 0, PRIORITY_TRESETALL, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Tasks created successfully" << endl << flush;

    /**************************************************************************************/
    /* Message queues creation                                                            */
    /**************************************************************************************/
    if ((err = rt_queue_create(&q_messageToMon, "q_messageToMon", sizeof (Message*)*50, Q_UNLIMITED, Q_FIFO)) < 0) {
        cerr << "Error msg queue create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    
    
    cout << "Queues created successfully" << endl << flush;

}

/**
 * @brief Démarrage des tâches
 */
void Tasks::Run() {
    rt_task_set_priority(NULL, T_LOPRIO);
    int err;

    if (err = rt_task_start(&th_server, (void(*)(void*)) & Tasks::ServerTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_sendToMon, (void(*)(void*)) & Tasks::SendToMonTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_receiveFromMon, (void(*)(void*)) & Tasks::ReceiveFromMonTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_openComRobot, (void(*)(void*)) & Tasks::OpenComRobot, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_startRobot, (void(*)(void*)) & Tasks::StartRobotTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_move, (void(*)(void*)) & Tasks::MoveTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_batteryLevel, (void(*)(void*)) & Tasks::BatteryLevelTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    
    if (err = rt_task_start(&th_WatchD, (void(*)(void*)) & Tasks::WatchDTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_watchCoRobot, (void(*)(void*)) & Tasks::WatchCoRobotTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_ResetAll, (void(*)(void*)) & Tasks::ResetAll, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }

    cout << "Tasks launched" << endl << flush;
}

/**
 * @brief Arrêt des tâches
 */
void Tasks::Stop() {
    monitor.Close();
    robot.Close();
}

/**
 */
void Tasks::Join() {
    cout << "Tasks synchronized" << endl << flush;
    rt_sem_broadcast(&sem_barrier);
    pause();
}

/**
 * @brief Thread handling server communication with the monitor.
 */
void Tasks::ServerTask(void *arg) {
    int status;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are started)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task server starts here                                                        */
    /**************************************************************************************/
    rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
    status = monitor.Open(SERVER_PORT);
    rt_mutex_release(&mutex_monitor);

    cout << "Open server on port " << (SERVER_PORT) << " (" << status << ")" << endl;

    if (status < 0) throw std::runtime_error {
        "Unable to start server on port " + std::to_string(SERVER_PORT)
        
    };
    monitor.AcceptClient(); // Wait the monitor client
    cout << "Rock'n'Roll baby, client accepted!" << endl << flush;
    rt_sem_broadcast(&sem_serverOk);
}

/**
 * @brief Thread sending data to monitor.
 */
void Tasks::SendToMonTask(void* arg) {
    Message *msg;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task sendToMon starts here                                                     */
    /**************************************************************************************/
    rt_sem_p(&sem_serverOk, TM_INFINITE);

    while (1) {
        cout << "wait msg to send" << endl << flush;
        msg = ReadInQueue(&q_messageToMon);
        cout << "Send msg to mon: " << msg->ToString() << endl << flush;
        rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
        monitor.Write(msg); // The message is deleted with the Write
        rt_mutex_release(&mutex_monitor);
    }
}

/**
 * @brief Thread receiving data from monitor.
 */
void Tasks::ReceiveFromMonTask(void *arg) {
    Message *msgRcv;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task receiveFromMon starts here                                                */
    /**************************************************************************************/
    rt_sem_p(&sem_serverOk, TM_INFINITE);
    cout << "Received message from monitor activated" << endl << flush;

    while (1) {
        msgRcv = monitor.Read();
        cout << "Rcv <= " << msgRcv->ToString() << endl << flush;

        if (msgRcv->CompareID(MESSAGE_MONITOR_LOST)) {
           delete(msgRcv);
           rt_sem_v(&sem_reset);
           
           //exit(-1);
        } 
        else if (msgRcv->CompareID(MESSAGE_ROBOT_COM_OPEN)) {
            rt_sem_v(&sem_openComRobot);
        } 
        else if (msgRcv->CompareID(MESSAGE_ROBOT_START_WITHOUT_WD)) {
            rt_sem_v(&sem_startRobot);
            rt_mutex_acquire(&mutex_robotMode, TM_INFINITE);
            robotMode = 1;
            rt_mutex_release(&mutex_robotMode);
        }
        else if (msgRcv->CompareID(MESSAGE_ROBOT_START_WITH_WD)) {
            rt_sem_v(&sem_startRobot);
            rt_mutex_acquire(&mutex_robotMode, TM_INFINITE);
            robotMode = 2;
            rt_mutex_release(&mutex_robotMode);
        } 
        else if (msgRcv->CompareID(MESSAGE_ROBOT_GO_FORWARD) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_BACKWARD) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_LEFT) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_RIGHT) ||
                msgRcv->CompareID(MESSAGE_ROBOT_STOP)) {

            rt_mutex_acquire(&mutex_move, TM_INFINITE);
            move = msgRcv->GetID();
            rt_mutex_release(&mutex_move);
        }
        delete(msgRcv); // mus be deleted manually, no consumer
    }
}

/**
 * @brief Thread opening communication with the robot.
 */
void Tasks::OpenComRobot(void *arg) {
    int status;
    int err;

    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task openComRobot starts here                                                  */
    /**************************************************************************************/
    while (1) {
        rt_sem_p(&sem_openComRobot, TM_INFINITE);
        cout << "Open serial com (";
        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
        status = robot.Open();
        rt_mutex_release(&mutex_robot);
        cout << status;
        cout << ")" << endl << flush;

        Message * msgSend;
        if (status < 0) {
            msgSend = new Message(MESSAGE_ANSWER_NACK);
            rt_sem_v(&sem_reset);
            
        } else {
            msgSend = new Message(MESSAGE_ANSWER_ACK);
            rt_sem_v(&sem_comRobotOK);
        }
        WriteInQueue(&q_messageToMon, msgSend); // msgSend will be deleted by sendToMon
    }
}

/**
 * @brief Thread starting the communication with the robot.
 */
void Tasks::StartRobotTask(void *arg) {
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task startRobot starts here                                                    */
    /**************************************************************************************/
    Message * msgSend;
    int mode = 0;  
    while (1) {
        
        rt_sem_p(&sem_startRobot, TM_INFINITE);
        
        rt_mutex_acquire(&mutex_robotMode, TM_INFINITE);
        mode = robotMode;
        rt_mutex_release(&mutex_robotMode);
        
        if(mode){ //mode=0 <=> default, means an error 
            
            if(mode==1){ //mode=1 <=> no WD
                cout << "Start robot without watchdog (";

                rt_mutex_acquire(&mutex_robot, TM_INFINITE);        
                msgSend = robot.Write(robot.StartWithoutWD());
                rt_mutex_release(&mutex_robot);

                cout << msgSend->GetID();
                cout << ")" << endl;
            }
            else if(mode==2){ //mode=2 <=> with WD
                cout << "Start robot with watchdog (";

                rt_mutex_acquire(&mutex_robot, TM_INFINITE);
                msgSend = robot.Write(robot.StartWithWD());
                rt_mutex_release(&mutex_robot);
                rt_sem_v(&sem_startWatchD);
                
                cout << msgSend->GetID();
                cout << ")" << endl; 
                
            }
        
            cout << "Start answer: " << msgSend->ToString() << endl << flush;
            WriteInQueue(&q_messageToMon, msgSend);  // msgSend will be deleted by sendToMon

            if (msgSend->GetID() == MESSAGE_ANSWER_ACK) {
                rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
                robotStarted = 1;
                rt_mutex_release(&mutex_robotStarted);
            }
        }
        //reset robot status to default (mode 0 means robot off)
        rt_mutex_acquire(&mutex_robotMode, TM_INFINITE);
        robotMode = 0;
        rt_mutex_release(&mutex_robotMode);
    }
}

/**
 * @brief Thread handling control of the robot.
 */
void Tasks::MoveTask(void *arg) {
    int rs;
    int cpMove;
    Message *msgReturn;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task starts here                                                               */
    /**************************************************************************************/
    rt_task_set_periodic(NULL, TM_NOW, 100000000);

    while (1) {
        rt_task_wait_period(NULL);
        cout << "Periodic movement update";
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        rs = robotStarted;
        rt_mutex_release(&mutex_robotStarted);
        if (rs == 1) {
            rt_mutex_acquire(&mutex_move, TM_INFINITE);
            cpMove = move;
            rt_mutex_release(&mutex_move);
            
            cout << " move: " << cpMove;
            
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            msgReturn = robot.Write(new Message((MessageID)cpMove));
            rt_mutex_release(&mutex_robot);
            
            cout << "Movement answer: " << msgReturn->ToString() << endl << flush;
            WriteInQueue(&q_messageToMon, msgReturn); 
        }
        cout << endl << flush;
    }
}

/**
 * @brief Thread handling control of the battery.
 */
void Tasks::BatteryLevelTask(void *arg) {
    int stateRob;
    int trueEmpty=0;//vérifier que la batterie est bien vide 
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task BatteryLevel starts here                                                    */
    /**************************************************************************************/
    
    
    rt_task_set_periodic(NULL, TM_NOW, 500000000);
    
    while(1) {
        rt_task_wait_period(NULL);
        
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        stateRob = robotStarted;
        rt_mutex_release(&mutex_robotStarted);
        if (stateRob) {
            MessageBattery * msg;
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            msg = (MessageBattery*)robot.Write(new Message(MESSAGE_ROBOT_BATTERY_GET));
            rt_mutex_release(&mutex_robot);
            WriteInQueue(&q_messageToMon, msg);
            
            //cout << to_string(((MessageBattery*) msg)->GetLevel())<<endl<<flush;
            
           if (to_string(((MessageBattery*) msg)->GetLevel())=="0")
           { 
               trueEmpty++;      //some errors make the battery appear empty -> double check to be sure
               if(trueEmpty==2){
                   cout << "Battery empty, robot shutting down" << endl << flush;
                rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
                robotStarted = 0;
                rt_mutex_release(&mutex_robotStarted);
                rt_mutex_acquire(&mutex_robot, TM_INFINITE);
                robot.Close();
                rt_mutex_release(&mutex_robot);
                rt_sem_v(&sem_openComRobot);
               }
                
                
           }
           else{trueEmpty=0;}
        }
    }
}

void Tasks::WatchDTask(void *arg){
    cout << "Start" << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task WatchD starts here                                                    */
    /**************************************************************************************/
    rt_sem_p(&sem_startWatchD, TM_INFINITE);
    rt_task_set_periodic(NULL, TM_NOW, 1000000000);
    
    Message * msgSend;
    int stateRob = 0;
    
    while(1){
        //Waiting for next cycle to start
        rt_task_wait_period(NULL);
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        stateRob = robotStarted;
        rt_mutex_release(&mutex_robotStarted);
        
        if(stateRob){//if robot is on, start sending reload message
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            msgSend = (Message*)robot.Write(new Message(MESSAGE_ROBOT_RELOAD_WD));
            
            rt_mutex_release(&mutex_robot);
            
        }
        
    }
    
    
}

void Tasks::WatchCoRobotTask(void *arg) {
    int num_error;
    Message * msgSend;
    Message *msgRob;
    int stateRob = 0;
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task WatchCoRobot starts here                                                    */
    /**************************************************************************************/
    
    rt_task_set_periodic(NULL, TM_NOW, 50000000); //0.5 seconds
    
    while(1) {
    rt_sem_p(&sem_comRobotOK, TM_INFINITE);    
    stateRob = 1;
        while(stateRob){
            rt_task_wait_period(NULL);
            cout << "Periodic robot connection checking update\n";


            rt_mutex_acquire(&mutex_err_count_robot, TM_INFINITE);
            num_error = err_count_robot;
            cout << "nb erreur: " << num_error << "\n" << endl << flush;
            rt_mutex_release(&mutex_err_count_robot);
            
            msgRob = ReadInQueue(&q_messageToMon);
            /*FAILED MESSAGE TEST*/
            if(msgRob->GetID() == MESSAGE_ANSWER_ROBOT_TIMEOUT ||
               msgRob->GetID() == MESSAGE_ANSWER_ROBOT_ERROR ||
               msgRob->GetID() == MESSAGE_ANSWER_ROBOT_UNKNOWN_COMMAND) 
            {
                rt_mutex_acquire(&mutex_err_count_robot, TM_INFINITE);
                err_count_robot++;
                rt_mutex_release(&mutex_err_count_robot);
                cout << "ERREUR de reception" << endl << flush;
            }
           
            
            else{
            rt_mutex_acquire(&mutex_err_count_robot, TM_INFINITE);
            err_count_robot=0;
            rt_mutex_release(&mutex_err_count_robot);
            }
            
            if(num_error >= 3)
            {   
                
                cout << "Connection lost with robot: closing" << endl << flush;
                
                rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
                robotStarted = 0;
                rt_mutex_release(&mutex_robotStarted);
                rt_mutex_acquire(&mutex_robot, TM_INFINITE);
                robot.Close();
                rt_mutex_release(&mutex_robot);
               
                stateRob = 0;
            }
        }
    }
}
    




/**
 * @brief reset all.
 */
void Tasks::ResetAll(void *arg) {
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task ResetAllTasks starts here                                                    */
    /**************************************************************************************/
    Message * msgSend;
    
    while(1) {
        rt_sem_p(&sem_reset, TM_INFINITE);
        cout << "Connexion perdue avec le moniteur" << endl << flush;
        
        cout << "Arret robot (";
        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
        msgSend = robot.Write(robot.Reset());
        rt_mutex_release(&mutex_robot);
        cout << msgSend->GetID();
        cout << ")" << endl;
        
        cout << "Fermeture connexion robot... ";
        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
        robot.Close();
        rt_mutex_release(&mutex_robot);
        cout << msgSend->GetID() << endl;
        
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        robotStarted = 0;
        rt_mutex_release(&mutex_robotStarted);  
        
        cout << "Fermeture connexion moniteur... ";
        rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
        monitor.Close();
        rt_mutex_release(&mutex_monitor);
        cout << msgSend->GetID() << endl;
        
       
        rt_queue_flush(&q_messageToMon);
    }
}

/**
 * Write a message in a given queue
 * @param queue Queue identifier
 * @param msg Message to be stored
 */
void Tasks::WriteInQueue(RT_QUEUE *queue, Message *msg) {
    int err;
    if ((err = rt_queue_write(queue, (const void *) &msg, sizeof ((const void *) &msg), Q_NORMAL)) < 0) {
        cerr << "Write in queue failed: " << strerror(-err) << endl << flush;
        throw std::runtime_error{"Error in write in queue"};
    }
}

/**
 * Read a message from a given queue, block if empty
 * @param queue Queue identifier
 * @return Message read
 */
Message *Tasks::ReadInQueue(RT_QUEUE *queue) {
    int err;
    Message *msg;

    if ((err = rt_queue_read(queue, &msg, sizeof ((void*) &msg), TM_INFINITE)) < 0) {
        cout << "Read in queue failed: " << strerror(-err) << endl << flush;
        throw std::runtime_error{"Error in read in queue"};
    }/** else {
        cout << "@msg :" << msg << endl << flush;
    } /**/

    return msg;
}

