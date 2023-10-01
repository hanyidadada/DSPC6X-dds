#ifndef MQTTWORK_H_
#define MQTTWORK_H_

#define UPPERTOPIC	"upper/subtopic"
#define NODESTATUS  "node/+/status"
#define NODERESULT  "node/+/result"
#define NODEMESSAGE  "node/+/"

#define MASTERSERIALTOPIC	"master/peripheral/serial"
#define MQTTHost	"10.2.25.21"
#define MQTTPort	1883
int MQTTWork(void);
#endif
