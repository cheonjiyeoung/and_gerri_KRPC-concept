from bosdyn.client import ResponseError, RpcError
from bosdyn.client.lease import Error as LeaseBaseError

def try_grpc(desc, thunk):
    try:
        return thunk()
    except (ResponseError, RpcError, LeaseBaseError) as err:
        print(f'Failed {desc}: {err}')
        return None

def try_grpc_async(desc, thunk):

    def on_future_done(fut):
        try:
            fut.result()
        except (ResponseError, RpcError, LeaseBaseError) as err:
            print(f'Failed {desc}: {err}')
            return None

    future = thunk()
    future.add_done_callback(on_future_done)