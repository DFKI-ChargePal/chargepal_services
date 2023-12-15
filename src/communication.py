from typing import Deque, Dict, List, Optional, Type
from types import TracebackType
from collections import defaultdict, deque
from multiprocessing.connection import Client, Listener
from threading import Condition, Thread


class Communication:
    SERVER_ADDRESS = ("192.168.158.25", 9000)

    def __init__(self, port: int) -> None:
        assert (
            1024 <= port <= 65535 and port != self.SERVER_ADDRESS[1]
        ), f"Robot port must be in 1024-65535 and different from server port {self.SERVER_ADDRESS[1]}."
        self.active = True
        self._messages: Deque[str] = deque()
        self._topic_messages: Dict[str, Deque[str]] = defaultdict(deque)
        self._condition = Condition()
        self._topic_conditions: Dict[str, Condition] = defaultdict(Condition)
        self.port = port
        self.robot_address = ("192.168.158.33", port)
        # Note: Send before listening to ensure server connection
        #  because otherwise listening gets stuck on an error.
        self.send("REQUEST_PORT")
        self._listener = Listener(self.robot_address)
        self._listener_thread = Thread(target=self.listen)
        self._listener_thread.start()

    def __enter__(self) -> "Communication":
        return self

    def __exit__(
        self,
        exception_type: Type[BaseException],
        exception_value: BaseException,
        traceback: TracebackType,
    ) -> None:
        self.shutdown()

    def send(self, message: str) -> None:
        """Establish connection to send one message to server."""
        try:
            client = Client(self.SERVER_ADDRESS)
            client.send(f"{self.port} {message}")
            client.close()
        except ConnectionRefusedError as e:
            raise e

    def listen(self) -> None:
        """Listen for server messages on this robot's dedicated port."""
        connection = self._listener.accept()
        while self.active:
            try:
                # Note: 08.12.2023 Cannot abort this connection cleanly
                #  without receiving a message from the server.
                message = str(connection.recv())
                topic = message.split()[0]
                if topic in self._topic_conditions.keys():
                    condition = self._topic_conditions[topic]
                    condition.acquire()
                    self._topic_messages[topic].append(message)
                    condition.notify_all()
                    condition.release()
                else:
                    self._condition.acquire()
                    self._messages.append(message)
                    self._condition.notify_all()
                    self._condition.release()
            except EOFError:
                pass
        connection.close()

    def pop_messages(self, topic: Optional[str] = None) -> List[str]:
        """Remove all messages for topic from its queue and return them."""
        if topic:
            self._topic_conditions[topic].acquire()
            messages = list(self._topic_messages[topic])
            self._topic_messages[topic].clear()
            self._topic_conditions[topic].release()
        else:
            self._condition.acquire()
            messages = list(self._messages)
            self._messages.clear()
            self._condition.release()
        return messages

    def wait_for_message(
        self,
        topic: Optional[str] = None,
        timeout: Optional[float] = None,
        discard_existing_messages: bool = True,
    ) -> Optional[str]:
        """
        Wait for message on topic or until a timeout occurs.
         If discard_existing_messages, clear message queue before waiting.
        """
        if topic:
            if topic not in self._topic_conditions.keys():
                condition = self._topic_conditions[topic] = Condition()
            else:
                condition = self._topic_conditions[topic]
            messages = self._topic_messages[topic]
        else:
            condition = self._condition
            message = self._messages
        condition.acquire()
        if discard_existing_messages:
            messages.clear()
        condition.wait(timeout)
        message = messages.popleft() if messages else None
        condition.release()
        return message

    def shutdown(self) -> None:
        if self.active:
            self.active = False
            Client(self.robot_address).close()
            self._listener.close()
            self._listener_thread.join()
