" YCM settings
let g:ycm_max_num_candidates = 5
let g:ycm_warning_symbol = '>'
let g:ycm_confirm_extra_conf = 0
let g:ycm_add_preview_to_completeopt = 1 
let g:ycm_autoclose_preview_window_after_completion = 1
let g:ycm_max_diagnostics_to_display = 0  " Reference: https://github.com/ycm-core/YouCompleteMe/issues/2392

" YCM Error & Warning Color Scheme
hi YcmErrorSection ctermbg=0 cterm=underline
hi YcmWarningSection ctermbg=220 guibg=#ffaf5f
