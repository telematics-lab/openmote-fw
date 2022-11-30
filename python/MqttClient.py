import logging
import threading
import queue

import paho.mqtt.client as mqtt

# Import logging configuration
logger = logging.getLogger(__name__)

class MqttClient(threading.Thread):
    def __init__(self, address = "127.0.0.1", port = 1883):
        logger.debug('init: Creating the MQTTClient object.')

        # Call thread constructor
        threading.Thread.__init__(self)

        # Lock to wait for connection
        self.is_connected = threading.Event()

        # Terminate thread event
        self.is_finished = threading.Event()

        # Message queue
        self.message_queue = queue.Queue()

        # Store the MQTT server, port and timeout
        self.mqtt_address = address
        self.mqtt_port    = port
        self.mqtt_timeout = 30

        self.mqtt_client = None
        self.mqtt_topics = {}

    def mqtt_on_connect(self, client, data, flags, rc):
        logger.debug("mqtt_on_connect: connected.")

        # Subscribe to topics
        for topic in self.mqtt_topics:
            self.mqtt_client.subscribe(topic)
            logger.debug("mqtt_on_connect: Subscribed to topic={}.".format(topic))

        # Set to signal connection
        self.is_connected.set()

    def mqtt_on_disconnect(self, client, data, rc):
        logger.debug("mqtt_on_disconnect: disconnected.")

        # Clear to signal unconnected
        self.is_connected.clear()

        # Connect the MQTT client to the broker
        self.mqtt_client.connect(self.mqtt_address, self.mqtt_port, self.mqtt_timeout)

    def mqtt_on_message(self, client, userdata, msg):
        logger.debug("mqtt_on_message: message received.")

        # Recover topic and message
        topic = msg.topic
        payload = msg.payload.decode('utf-8')

        # Execute message callback
        self.mqtt_topics[topic](topic, payload)

    def start(self):
        logger.debug("start: Starting the MQTTClient.")

        # Create MQTT client
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.mqtt_on_connect
        self.mqtt_client.on_disconnect = self.mqtt_on_disconnect
        self.mqtt_client.on_message = self.mqtt_on_message

        # Call thread start
        threading.Thread.start(self)

    def run(self):
        logger.debug("run: Running the MQTTClient.")

        # Connect the MQTT client to the broker
        self.mqtt_client.connect(self.mqtt_address, self.mqtt_port, self.mqtt_timeout)

        # Start the MQTT client
        self.mqtt_client.loop_start()

        # Run until finish condition is set
        while (not self.is_finished.isSet()):
            # Wait for connection
            while (self.is_connected.is_set()):
                try:
                    # Recover element from queue
                    element = self.message_queue.get(block=True, timeout=0.1)

                    # If we received an element
                    if (element is not None):
                        # Recover topic and message from element
                        topic = element['topic']
                        message = element['message']

                        # Log the message that has been send
                        logger.info("run: Sending topic={}, message={}.".format(topic, message))

                        # Publish the JSON message to the MQTT broker
                        self.mqtt_client.publish(topic, message)
                except queue.Empty:
                    pass

        # Stop the MQTT thread
        self.mqtt_client.loop_stop()

        logger.info("run: Stopped the MQTTClient.")

    def stop(self):
        logger.debug("stop: Stopping the MQTTClient.")
        
        # Terminates the thread
        self.is_finished.set()

    def add_topic(self, topic=None, callback=None):
        logger.debug("add_topic: Adding topic={}.".format(topic))

        # Subscribe to topic
        self.mqtt_topics[topic] = callback

    def add_topics(self, topics=None, callbacks=None):
        for t, c in zip(topics, callbacks):
            self.add_topic(topic=t, callback=c)

    def send_message(self, topic, message):
        logger.debug("send_message: Adding message to queue.")

        # Create message
        element = {"topic": topic, "message": message}

        # Send element to message queue
        self.message_queue.put(element)

        logger.debug("send_message: Added message to queue.")
