from __future__ import annotations

from collections.abc import Iterator
from concurrent import futures

import asr_pb2
import audio_pb2
import grpc
import map_pb2
import robonix_contracts_pb2_grpc as contracts_grpc
import speech_pb2
import tts_pb2


class Provider:
    def grpc(self, _contract_id: str, *, description: str = ""):
        del description

        def decorate(fn):
            return fn

        return decorate


class Synthesizer:
    def generate(self, text: str, *, voice: str):
        del text, voice
        yield audio_pb2.AudioChunk(data=b"pcm", sequence=0)


class Detector:
    def detect(self, chunks) -> str:
        return "robonix" if any(chunks) else ""


class Decoder:
    def __init__(self) -> None:
        self._data = bytearray()

    def feed(self, data: bytes) -> None:
        self._data.extend(data)

    def partial_text(self) -> str:
        return "partial" if self._data else ""

    def final_text(self) -> str:
        return "final"


provider = Provider()
synthesizer = Synthesizer()
detector = Detector()
StreamingDecoder = Decoder


@provider.grpc("robonix/service/map/get_mode")
def get_mode(
    request: map_pb2.GetMode_Request,
    context: grpc.ServicerContext,
) -> map_pb2.GetMode_Response:
    del request, context
    return map_pb2.GetMode_Response(ok=True, mode="mapping", detail="")


@provider.grpc("robonix/service/speech/tts_stream")
def synthesize_stream(
    request: tts_pb2.SynthesizeStream_Request,
    context: grpc.ServicerContext,
) -> Iterator[tts_pb2.SynthesizeAudioChunk]:
    for chunk in synthesizer.generate(request.text, voice=request.voice):
        if not context.is_active():
            return
        yield tts_pb2.SynthesizeAudioChunk(
            chunk=chunk,
            encoding="pcm_s16le",
            sample_rate_hz=16000,
        )


@provider.grpc("robonix/service/speech/wake_word")
def detect_wake_word(
    request_iterator: Iterator[audio_pb2.AudioChunk],
    context: grpc.ServicerContext,
) -> speech_pb2.DetectWakeWord_Response:
    keyword = detector.detect(
        bytes(chunk.data)
        for chunk in request_iterator
        if context.is_active() and chunk.data
    )
    return speech_pb2.DetectWakeWord_Response(
        detected=bool(keyword), keyword=keyword or ""
    )


@provider.grpc("robonix/service/speech/asr_stream")
def recognize_stream(
    request_iterator: Iterator[asr_pb2.AsrAudioChunk],
    context: grpc.ServicerContext,
) -> Iterator[asr_pb2.RecognizeStreamEvent]:
    decoder = StreamingDecoder()
    for request in request_iterator:
        if not context.is_active():
            return
        decoder.feed(request.chunk.data)
        partial = decoder.partial_text()
        if partial:
            yield asr_pb2.RecognizeStreamEvent(
                event_type=asr_pb2.RecognizeStreamEvent.PARTIAL,
                text=partial,
                is_final=False,
            )
    yield asr_pb2.RecognizeStreamEvent(
        event_type=asr_pb2.RecognizeStreamEvent.FINAL,
        text=decoder.final_text(),
        is_final=True,
    )


class MapServicer(contracts_grpc.RobonixServiceMapGetModeServicer):
    def GetMode(self, request, context):
        return get_mode(request, context)


class TtsServicer(contracts_grpc.RobonixServiceSpeechTtsStreamServicer):
    def SynthesizeStream(self, request, context):
        return synthesize_stream(request, context)


class WakeWordServicer(contracts_grpc.RobonixServiceSpeechWakeWordServicer):
    def DetectWakeWord(self, request_iterator, context):
        return detect_wake_word(request_iterator, context)


class AsrServicer(contracts_grpc.RobonixServiceSpeechAsrStreamServicer):
    def RecognizeStream(self, request_iterator, context):
        return recognize_stream(request_iterator, context)


server = grpc.server(futures.ThreadPoolExecutor(max_workers=4))
contracts_grpc.add_RobonixServiceMapGetModeServicer_to_server(MapServicer(), server)
contracts_grpc.add_RobonixServiceSpeechTtsStreamServicer_to_server(TtsServicer(), server)
contracts_grpc.add_RobonixServiceSpeechWakeWordServicer_to_server(
    WakeWordServicer(), server
)
contracts_grpc.add_RobonixServiceSpeechAsrStreamServicer_to_server(AsrServicer(), server)
port = server.add_insecure_port("127.0.0.1:0")
assert port > 0
server.start()

channel = grpc.insecure_channel(f"127.0.0.1:{port}")
try:
    map_stub = contracts_grpc.RobonixServiceMapGetModeStub(channel)
    map_response = map_stub.GetMode(map_pb2.GetMode_Request(), timeout=5.0)
    assert map_response.mode == "mapping"

    tts_stub = contracts_grpc.RobonixServiceSpeechTtsStreamStub(channel)
    tts_events = list(
        tts_stub.SynthesizeStream(
            tts_pb2.SynthesizeStream_Request(text="hi"), timeout=5.0
        )
    )
    assert tts_events[0].chunk.data == b"pcm"

    wake_stub = contracts_grpc.RobonixServiceSpeechWakeWordStub(channel)
    wake_response = wake_stub.DetectWakeWord(
        iter([audio_pb2.AudioChunk(data=b"pcm")]), timeout=5.0
    )
    assert wake_response.detected and wake_response.keyword == "robonix"

    asr_stub = contracts_grpc.RobonixServiceSpeechAsrStreamStub(channel)
    asr_events = list(
        asr_stub.RecognizeStream(
            iter(
                [
                    asr_pb2.AsrAudioChunk(
                        chunk=audio_pb2.AudioChunk(data=b"pcm")
                    )
                ]
            ),
            timeout=5.0,
        )
    )
    assert [event.event_type for event in asr_events] == [0, 1]
finally:
    channel.close()
    server.stop(grace=0).wait(timeout=5.0)

print("all four documented gRPC provider and caller shapes executed over loopback")
