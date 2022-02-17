#! /usr/bin/env python3
# Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

def wait_for_service(in_node, in_service, in_service_name, in_timeout=5.0):
    print(f'Waiting for service [{in_service_name}]...')
    cli = in_node.create_client(in_service, in_service_name)
    if not cli.wait_for_service(timeout_sec=in_timeout):
        in_node.get_logger().error(f'{in_service}[{in_service_name}] service not available')
    return cli