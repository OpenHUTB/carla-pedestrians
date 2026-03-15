from typing import Dict, Iterable
import numpy
import torch
import pandas


class MixedDataset(torch.utils.data.ConcatDataset):
    def __init__(
        self,
        datasets,
        skip_metadata: bool = False,
        proportions: Iterable[float] = None,
        mappings: Dict[str, str] = None,
        **kwargs
    ):
        if proportions is None:  # use all available data
            subsets = datasets
        elif all([(p == 0 or p == -1) for p in proportions]):  # use whole dataset or none
            subsets = [datasets[i] for i, p in enumerate(proportions) if p != 0]
        else:  # use a subset of the dataset
            subsets = []
            lengths = [len(dataset) for dataset in datasets]
            possible_total = min([lengths[i] / p if p != 0 else float('inf')
                                 for i, p in enumerate(proportions)])
            for i, p in enumerate(proportions):
                if p == 0:
                    continue
                else:
                    subset_size = int(possible_total * p)
                    subsets.append(
                        torch.utils.data.Subset(
                            datasets[i],
                            numpy.random.choice(
                                range(lengths[i]),
                                subset_size,
                                replace=False
                            )
                        )
                    )

        super().__init__(subsets)

        self._mappings = mappings
        self._inverse_mappings = {v: k for k,
                                  v in mappings.items()} if mappings else None

        # figure out common targets
        self._targets_template = {}
        first_items = [dataset[0][1] for dataset in datasets]
        mappings_keys = tuple(self._mappings.keys()) if self._mappings else tuple()
        all_keys = set(
            [k for targets in first_items for k in targets.keys() if k not in mappings_keys])
        for key in all_keys:
            shapes = numpy.array([tuple(targets[key].shape)
                                  for targets in first_items if key in targets])
            assert numpy.all(
                shapes == shapes[0]), f'{key} has different shapes in different datasets: {str(list(shapes))}'

            dtypes = [targets[key].numpy().dtype for targets in first_items if key in targets]
            common_dtype = numpy.find_common_type(dtypes, [])
            self._targets_template[key] = (common_dtype, tuple(shapes[0]))

        # if metadata is loaded, try to figure out common data types/fields
        self._meta_template = None
        if not skip_metadata:
            first_items = [dataset[0][2] for dataset in subsets]
            common_df = pandas.DataFrame.from_dict(first_items, orient='columns')
            self._meta_template = {
                k: v if v != numpy.dtype('object') else numpy.dtype('str')
                for (k, v) in common_df.dtypes.to_dict().items()
                if k not in mappings_keys
            }

    def __getitem__(self, index):
        projection_2d, targets, meta = super().__getitem__(index)

        common_targets = {}
        for key, (template_dtype, template_shape) in self._targets_template.items():
            mapped_key = self._inverse_mappings.get(
                key, None) if self._inverse_mappings else None
            if key in targets:
                common_targets[key] = targets[key]
            elif mapped_key in targets:
                common_targets[key] = targets[mapped_key]
            else:
                common_targets[key] = torch.from_numpy(numpy.full(
                    template_shape,
                    numpy.nan,  # TODO: NaNs or zeros?
                    dtype=template_dtype
                ))

        common_meta = {}
        if self._meta_template is not None:
            for key, template_dtype in self._meta_template.items():
                mapped_key = self._inverse_mappings.get(
                    key, None) if self._inverse_mappings else None
                if key in meta:
                    common_meta[key] = numpy.array(
                        [meta[key]], dtype=template_dtype).item()
                elif mapped_key in meta:
                    common_meta[key] = numpy.array(
                        [meta[mapped_key]], dtype=template_dtype).item()
                else:
                    common_meta[key] = numpy.array(
                        [numpy.nan], dtype=template_dtype).item()

        return projection_2d, common_targets, common_meta
