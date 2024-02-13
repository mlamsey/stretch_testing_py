import zmq

def open_socket_listener(socket_ip: str) -> tuple:
    """
    Opens a zmq socket to listen for messages.

    Args:
        socket_ip (str): The IP address of the socket.

    Returns:
        tuple: A tuple containing the socket and poller objects.
    """

    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.bind(socket_ip)
    socket.setsockopt_string(zmq.SUBSCRIBE, '')

    poller = zmq.Poller()
    poller.register(socket, zmq.POLLIN)
    return socket, poller