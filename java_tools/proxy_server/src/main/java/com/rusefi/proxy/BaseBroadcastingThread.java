package com.rusefi.proxy;

import com.opensr5.Logger;
import com.rusefi.NamedThreadFactory;
import com.rusefi.Timeouts;
import com.rusefi.binaryprotocol.IncomingDataBuffer;
import com.rusefi.config.generated.Fields;
import com.rusefi.io.commands.HelloCommand;
import com.rusefi.io.tcp.BinaryProtocolServer;
import com.rusefi.io.tcp.TcpIoStream;
import com.rusefi.server.SessionDetails;

import java.io.IOException;
import java.net.Socket;

import static com.rusefi.io.tcp.BinaryProtocolServer.getPacketLength;
import static com.rusefi.io.tcp.BinaryProtocolServer.readPromisedBytes;

public class BaseBroadcastingThread {
    private static final NamedThreadFactory BASE_BROADCASTING_THREAD = new NamedThreadFactory("BaseBroadcastingThread");
    // we expect server to at least request output channels once in a while
    private static final int IO_TIMEOUT = 600 * Timeouts.SECOND;
    private final Thread thread;

    @SuppressWarnings("InfiniteLoopStatement")
    public BaseBroadcastingThread(Socket socket, SessionDetails sessionDetails, Logger logger) throws IOException {
        TcpIoStream stream = new TcpIoStream("[broadcast] ", logger, socket);
        IncomingDataBuffer in = stream.getDataBuffer();

        thread = BASE_BROADCASTING_THREAD.newThread(() -> {
            try {
                while (true) {
                    int length = getPacketLength(in, () -> {
                        throw new UnsupportedOperationException();
                    }, IO_TIMEOUT);
                    BinaryProtocolServer.Packet packet = readPromisedBytes(in, length);
                    byte[] payload = packet.getPacket();

                    byte command = payload[0];

                    if (command == Fields.TS_HELLO_COMMAND) {
                        // respond on hello request with information about session
                        logger.info("Sending to controller connector@proxy: " + sessionDetails);
                        new HelloCommand(logger, sessionDetails.toJson()).handle(stream);
                    } else {
                        handleCommand(packet, stream);
                    }
                }
            } catch (IOException e) {
                logger.error("exiting thread " + e);
            }
        });
    }

    protected void handleCommand(BinaryProtocolServer.Packet packet, TcpIoStream stream) throws IOException {
    }

    public void start() {
        thread.start();
    }
}