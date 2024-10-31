class NotAvailableException(Exception):
    """
    Exception raised when a resource is not available, e.g. when it is optional and dependencies are not installed.
    """

    def __init__(self, functionality_name: str, optional_group_name: str):
        self.functionality_name = functionality_name
        self.optional_group_name = optional_group_name
        self.message = f"{self.functionality_name} is not available, please run `pip install pedestrians-video-2-carla[{self.optional_group_name}]`."
        super().__init__(self.message)
