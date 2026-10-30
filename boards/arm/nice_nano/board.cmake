board_runner_args(jlink "--device=nRF52840_xxAA" "--speed=4000")
board_runner_args(uf2 "--board-id=nice!nano")

include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
include(${ZEPHYR_BASE}/boards/common/uf2.board.cmake)
